import os
import warnings
warnings.filterwarnings('ignore')
# 添加conda环境的lib路径到LD_LIBRARY_PATH
conda_env_path = "/data/zhangmeng/tujy24/my_software/anaconda3/envs/yolov8_test"
if os.path.exists(conda_env_path):
    lib_path = os.path.join(conda_env_path, "lib")
    os.environ['LD_LIBRARY_PATH'] = lib_path + ':' + os.environ.get('LD_LIBRARY_PATH', '')
# 禁用GPU以解决CUDA兼容性问题
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # 只显示错误日志

import pandas as pd
import numpy as np
import tensorflow as tf
from keras.models import Sequential
from keras.layers import Dense, Activation
from keras.optimizers import Adam
import paho.mqtt.client as mqtt
import threading
import time
from sklearn.linear_model import Ridge
from sklearn.model_selection import LeaveOneOut, GridSearchCV
from sklearn.metrics import mean_squared_error, r2_score, mean_absolute_error
from sklearn.preprocessing import RobustScaler
from sklearn.pipeline import make_pipeline
from sklearn.feature_selection import RFE
#================================================================#
class BloodGlucosePredictor:
    """优化后的血糖浓度预测模型（保留四个特征）"""
    def __init__(self, alpha=1.0, use_rfe=False):
        """
        初始化模型
        :param alpha: 岭回归正则化强度
        :param use_rfe: 是否使用递归特征消除
        """
        self.alpha = alpha
        self.model = None
        self.scaler = RobustScaler()  # 使用鲁棒缩放器处理异常值
        self.use_rfe = use_rfe
        self.loo_predictions = None
        self.coef_ = None
        self.intercept_ = None  
    def fit(self, X, y):
        """使用留一法训练模型并收集预测结果"""
        n_samples = X.shape[0]
        self.loo_predictions = np.zeros(n_samples)
        # 存储所有模型的系数和截距
        all_coefs = []
        all_intercepts = []
        # 使用留一法交叉验证
        loo = LeaveOneOut()
        for train_index, test_index in loo.split(X):
            X_train, X_test = X[train_index], X[test_index]
            y_train = y[train_index]
            # 缩放特征（仅使用训练数据拟合缩放器）
            X_train_scaled = self.scaler.fit_transform(X_train)
            X_test_scaled = self.scaler.transform(X_test)
            # 创建并训练岭回归模型
            model = Ridge(alpha=self.alpha, fit_intercept=True)
            # 可选：递归特征消除
            if self.use_rfe:
                selector = RFE(model, n_features_to_select=4, step=1)
                selector.fit(X_train_scaled, y_train)
                model = selector.estimator_
            model.fit(X_train_scaled, y_train)
            # 存储系数和截距
            all_coefs.append(model.coef_)
            all_intercepts.append(model.intercept_)
            # 预测并存储结果
            y_pred = model.predict(X_test_scaled)
            self.loo_predictions[test_index] = y_pred[0]
        # 计算平均系数和截距作为最终模型
        self.coef_ = np.mean(all_coefs, axis=0)
        self.intercept_ = np.mean(all_intercepts)
        # 创建最终模型
        X_scaled = self.scaler.fit_transform(X)
        self.model = Ridge(alpha=self.alpha, fit_intercept=True)
        self.model.fit(X_scaled, y)
        return self
    def predict(self, X):
        """使用最终模型进行预测"""
        X_scaled = self.scaler.transform(X)
        return self.model.predict(X_scaled)
    def evaluate_loo(self, y_true):
        """评估留一法预测结果"""
        mse = mean_squared_error(y_true, self.loo_predictions)
        rmse = np.sqrt(mse)
        r2 = r2_score(y_true, self.loo_predictions)
        mae = mean_absolute_error(y_true, self.loo_predictions)
        # 计算平均绝对百分比误差 (MAPE)
        absolute_percentage_errors = np.abs((y_true - self.loo_predictions) / y_true) * 100
        mape = np.mean(absolute_percentage_errors)
        return mse, rmse, r2, mae, mape, absolute_percentage_errors
    def print_coefficients(self):
        """打印模型系数"""
        print("\n最终模型系数:")
        print(f"截距项: {self.intercept_:.6f}")
        print(f"BMR 系数: {self.coef_[0]:.6f}")
        print(f"SPO2 系数: {self.coef_[1]:.6f}")
        print(f"PF 系数: {self.coef_[2]:.6f}")
        print(f"BV 系数: {self.coef_[3]:.6f}")
        
        print("\n模型方程:")
        print(f"GLU = {self.intercept_:.6f} + {self.coef_[0]:.6f}*BMR + {self.coef_[1]:.6f}*SPO2 + {self.coef_[2]:.6f}*PF + {self.coef_[3]:.6f}*BV")

# 数据加载
def load_data(file_path):
    """从Excel文件加载数据"""
    df = pd.read_excel(file_path)
    # 检查必要列
    required_columns = ['BMR', 'SPO2', 'PF', 'BV', 'GLU']
    missing = [col for col in required_columns if col not in df.columns]
    if missing:
        raise ValueError(f"缺少列: {', '.join(missing)}")
    # 提取数据
    X = df[['BMR', 'SPO2', 'PF', 'BV']].values
    y = df['GLU'].values
    # 数据摘要
    print("\n数据摘要:")
    print(f"样本数量: {X.shape[0]}")
    print(f"特征数量: {X.shape[1]}")
    print(df[required_columns].describe().round(2))
    return X, y, df

# 优化正则化参数
def optimize_alpha(X, y):
    """优化岭回归的正则化参数alpha"""
    # 创建管道：缩放 + 岭回归
    pipeline = make_pipeline(
        RobustScaler(),
        Ridge(fit_intercept=True)
    )
    # 参数网格
    param_grid = {'ridge__alpha': np.logspace(-3, 3, 20)}
    # 网格搜索（使用留一法交叉验证）
    grid_search = GridSearchCV(
        pipeline, 
        param_grid, 
        cv=LeaveOneOut(),
        scoring='neg_mean_absolute_error',
        n_jobs=-1
    )
    grid_search.fit(X, y)
    print(f"\n最佳alpha: {grid_search.best_params_['ridge__alpha']:.4f}")
    print(f"最佳MAE: {-grid_search.best_score_:.4f}")
    return grid_search.best_params_['ridge__alpha']

# 主程序
def Linear():
    try:
        # 加载数据
        FILE_PATH = "250702data.xlsx"
        X, y, df = load_data(FILE_PATH)
        # 优化正则化参数
        best_alpha = optimize_alpha(X, y)
        # 创建并训练模型（使用优化后的alpha）
        print("\n训练模型中...")
        predictor = BloodGlucosePredictor(alpha=best_alpha, use_rfe=False)
        predictor.fit(X, y)
        # 评估留一法结果
        print("\n留一法交叉验证结果:")
        mse, rmse, r2, mae, mape, absolute_percentage_errors = predictor.evaluate_loo(y)
        print(f"均方误差 (MSE): {mse:.4f}")
        print(f"均方根误差 (RMSE): {rmse:.4f}")
        print(f"平均绝对误差 (MAE): {mae:.4f}")
        print(f"平均绝对百分比误差 (MAPE): {mape:.2f}%")
        print(f"决定系数 (R²): {r2:.4f}")
        # 打印模型系数
        predictor.print_coefficients()
        # 添加预测结果和误差到数据框
        df['Predicted_GLU'] = predictor.loo_predictions
        df['Error'] = df['GLU'] - df['Predicted_GLU']
        df['Absolute_Error'] = np.abs(df['Error'])
        df['Percentage_Error'] = np.abs(df['Error'] / df['GLU']) * 100
        # 打印预测结果
        print("\n详细预测结果 (血糖浓度单位: mg/dL):")
        print(df[['BMR', 'SPO2', 'PF', 'BV', 'GLU', 'Predicted_GLU', 
                 'Error', 'Percentage_Error']].round(4))
        # 计算并显示误差统计
        max_error_idx = df['Absolute_Error'].idxmax()
        min_error_idx = df['Absolute_Error'].idxmin()
        print("\n误差分析:")
        print(f"最大绝对误差: {df.loc[max_error_idx, 'Absolute_Error']:.4f} mg/dL")
        print(f"最小绝对误差: {df.loc[min_error_idx, 'Absolute_Error']:.4f} mg/dL")
        print(f"最大百分比误差: {df.loc[max_error_idx, 'Percentage_Error']:.2f}%")
        print(f"最小百分比误差: {df.loc[min_error_idx, 'Percentage_Error']:.2f}%")
        # 按百分比误差排序
        sorted_by_error = df.sort_values('Percentage_Error', ascending=False)
        print("\n预测误差最大的5个样本:")
        print(sorted_by_error[['BMR', 'SPO2', 'PF', 'BV', 'GLU', 'Predicted_GLU', 
                             'Percentage_Error']].head(5).round(4))
        # 保存结果
        df.to_excel("optimized_glucose_prediction_results.xlsx", index=False)
        print("\n预测结果已保存到 optimized_glucose_prediction_results.xlsx")
        # 最终模型在全体数据上的表现
        final_predictions = predictor.predict(X)
        final_r2 = r2_score(y, final_predictions)
        print(f"\n最终模型在全体数据上的 R²: {final_r2:.4f}")
    except Exception as e:
        print(f"错误: {str(e)}")
        import traceback
        traceback.print_exc()
#================================================================#
def BP():
    # 读取Excel数据
    file_path = '250702data.xlsx'
    original_data = pd.read_excel(file_path)
    data = original_data.loc[:, ['BMR', 'SPO2', 'PF', 'BV', 'GLU']].values
    # 提取特征和目标值
    X = data[:, -5:-1]
    y_true = data[:, -1:]
    model = Sequential()
    model.add(Dense(8, input_dim = 4, use_bias=True))
    model.add(Activation('linear'))
    model.add(Dense(16, use_bias=True))
    model.add(Activation('linear')); 
    model.add(Dense(8, use_bias=True))
    model.add(Activation('linear')); 
    model.add(Dense(1, use_bias=True))
    model.add(Activation('linear'))
    # 自定义学习率
    optimizer = Adam(learning_rate=0.01)  # 根据文献调整学习率
    # 编译模型
    model.compile(loss='mean_squared_error', optimizer=optimizer)
    # 训练模型
    history = model.fit(X, y_true, epochs=1000, batch_size=1, verbose=0)
    # 使用模型进行预测
    y_pred = model.predict(X)
    # 计算误差百分比
    errors = np.abs((y_pred - y_true) / y_true) * 100
    average_error = np.mean(errors)
    # 确保所有数值都是一维且长度相同
    sample_indices = range(1, len(data) + 1)
    true_glucose = y_true.flatten()
    predicted_glucose = y_pred.flatten()
    error_percentages = errors.flatten()
    # 创建结果DataFrame
    results = pd.DataFrame({
        '样本序号': sample_indices,
        '真实血糖': true_glucose,
        '预测血糖': predicted_glucose,
        '误差百分比(%)': error_percentages
    })
    # 添加平均误差行
    average_error_row = pd.DataFrame([{'样本序号': '平均值', 
                                    '真实血糖': np.nan, 
                                    '预测血糖': np.nan, 
                                    '误差百分比(%)': average_error}])
    results = pd.concat([results, average_error_row], ignore_index=True)
    print("="*50)
    print("血糖预测结果及误差分析 (BP神经网络):")
    print(results.to_string(index=False))
    print("="*50)
    print(f"平均绝对误差百分比: {average_error:.2f}%")
    print(f"训练完成，最终损失值: {history.history['loss'][-1]:.4f}")
    # 将结果保存到Excel
    results.to_excel('nn_glucose_prediction_results.xlsx', index=False)
    print("预测结果已保存到 nn_glucose_prediction_results.xlsx")
    # 保存模型
    model.save('glucose_prediction_model.h5')
    print("神经网络模型已保存为 glucose_prediction_model.h5")
    print("输入形状:", model.input_shape)   # 应为 (None, 4)
    print("输出形状:", model.output_shape)  # 应为 (None, 1)
#================================================================#
# MQTT连接参数
client_id = f"k2ac35IRoHv.PC_train|securemode=2,signmethod=hmacsha256,timestamp=1750959430461|"
username = f"PC_train&k2ac35IRoHv"
password = "7e93f20f10e37e47c72e014d13faf0f4a1b1134b76682ba6f3e92821ffbd4898"
# 构建MQTT连接地址
broker_address = f"iot-06z00bjpwkwaqbv.mqtt.iothub.aliyuncs.com"
port = 1883

# 发送消息的函数
def send_message(client, topic, payload):
    try:
        result = client.publish(topic, payload)
        print(f"已发送消息到 {topic}: {payload}")
        result.wait_for_publish()  # 可选：等待发布完成
    except Exception as e:
        print(f"发送消息失败: {e}")

def on_connect(client, userdata, flags, rc):
    print(f"已经连接到BL618设备")
    # 订阅另一个设备上报数据的主题
    client.subscribe(f"/k2ac35IRoHv/PC_train/user/data_BL618_to_PC")

def on_message(client, userdata, msg):
    print(f"收到对应数据：{msg.topic} {str(msg.payload)}")
    print(f"\r\n==================开始参数更新=================\r\n")
    Linear()
    BP()
    print(f"\r\n==================参数更新完成==================\r\n")
    topic = f"/k2ac35IRoHv/PC_train/user/data_PC_to_BL618"
    print(f"已发送新参数到 {topic}")
# 用于监听用户输入并发送消息的函数
def input_loop(client):
    time.sleep(1)
    while True:
        try:
            topic = f"/k2ac35IRoHv/PC_train/user/data_PC_to_BL618"
            if topic.lower() == 'q':
                break
            payload = input("请输入您此时的真实血糖数据: ")
            send_message(client, topic, payload)
        except KeyboardInterrupt:
            break

# 创建MQTT客户端
mqtt_client = mqtt.Client(client_id=client_id)
mqtt_client.username_pw_set(username, password=password)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

# 连接并启动循环
try:
    print(f"\r\n==================嵌入式大赛==================\r\n")
    mqtt_client.connect(broker_address, port, 60)
    # 启动网络循环（后台接收）
    mqtt_client.loop_start()
    # 开启用户输入线程（前台发送）
    sender_thread = threading.Thread(target=input_loop, args=(mqtt_client,))
    sender_thread.start()
    # 等待线程结束
    sender_thread.join()
except Exception as e:
    print(f"发生错误: {e}")
finally:
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    print("MQTT客户端已关闭。")