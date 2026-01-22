#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口夹爪控制程序
通过串口发送16进制消息控制夹爪位置和速度
消息格式：帧头 + 命令头 + 位置 + 时间 + 速度 + 校验码
帧头 (FF FF) 不参与校验码计算
其余部分 (命令头 + 位置 + 时间 + 速度) 参与校验码计算

支持百分比输入：
- 位置：0-100% 对应 1600-3200
- 速度：0-100% 对应 0-1000

支持智能电流和位置监视：
- 电流监视命令：FF FF 0A 04 02 45 02 A8
- 位置监视命令：FF FF 0A 04 02 38 02 B5
- 当电流超过0.45A时，自动发送停止命令并终止监视
- 当位置稳定时（连续3次检测位置变化<5），自动终止监视
- 监视频率：默认每0.1秒检测一次
- 无需固定等待时间，智能判断运动完成
"""

import serial
import time
import struct
import threading

class GripperController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000, timeout=1):
        """
        初始化夹爪控制器
        :param port: 串口设备名
        :param baudrate: 波特率
        :param timeout: 超时时间
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.max_position = 3200
        self.min_position = 1600
        self.max_speed = 1000
        self.min_speed = 0
        
        # 监视相关变量
        self.monitoring = False
        self.current_threshold = 0.2  # 电流阈值 (A)
        self.monitor_thread = None
        self.last_position_command = None
        self.last_speed_command = None
        self.monitor_frequency = 0.05  # 监视频率 (秒)

        # 起始号固定值分为两段
        self.frame_header = [0xFF, 0xFF]  # 帧头，不参与校验码计算
        self.command_header = [0x0A, 0x09, 0x03, 0x2A]  # 命令头，参与校验码计算
        
    def connect(self):
        """连接串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"成功连接到串口: {self.port}")
            return True
        except Exception as e:
            print(f"连接串口失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("串口连接已断开")
    
    def position_percent_to_value(self, position_percent):
        """
        将位置百分比(0-100)转换为实际位置值(1600-3200)
        :param position_percent: 位置百分比 (0-100)
        :return: 实际位置值 (1600-3200)
        """
        if position_percent < 0 or position_percent > 100:
            raise ValueError(f"位置百分比必须在0-100范围内，当前值: {position_percent}")
        
        # 线性插值：0% -> 1600, 100% -> 3200
        position_value = int(self.min_position + (position_percent / 100.0) * (self.max_position - self.min_position))
        return position_value
    
    def position_to_hex(self, position):
        """
        将位置值(1600-3200)转换为16进制字节
        :param position: 位置值 (1600-3200)
        :return: 位置的16进制字节列表 [低字节, 高字节]
        """
        if position < 1600 or position > 3200:
            raise ValueError(f"位置值必须在1600-3200范围内，当前值: {position}")
        
        # 转换为16位整数的字节表示（小端序：低八位在前，高八位在后）
        position_bytes = struct.pack('<H', position)  # 小端序，无符号短整型
        return list(position_bytes)
    
    def speed_percent_to_value(self, speed_percent):
        """
        将速度百分比(0-100)转换为实际速度值(0-1000)
        :param speed_percent: 速度百分比 (0-100)
        :return: 实际速度值 (0-1000)
        """
        if speed_percent < 0 or speed_percent > 100:
            raise ValueError(f"速度百分比必须在0-100范围内，当前值: {speed_percent}")
        
        # 线性映射：0% -> 0, 100% -> 1000
        speed_value = int((speed_percent / 100.0) * self.max_speed)
        return speed_value
    
    def speed_to_hex(self, speed):
        """
        将速度值(0-1000)转换为16进制字节
        :param speed: 速度值 (0-1000)
        :return: 速度的16进制字节列表 [低字节, 高字节]
        """
        if speed < 0 or speed > 1000:
            raise ValueError(f"速度值必须在0-1000范围内，当前值: {speed}")
        
        # 转换为16位整数的字节表示（小端序：低八位在前，高八位在后）
        speed_bytes = struct.pack('<H', speed)  # 小端序，无符号短整型
        return list(speed_bytes)
    
    def time_to_hex(self, time_value=0):
        """
        将时间值转换为16进制字节
        :param time_value: 时间值，默认为0
        :return: 时间的16进制字节列表 [低字节, 高字节]
        """
        # 转换为16位整数的字节表示（小端序：低八位在前，高八位在后）
        time_bytes = struct.pack('<H', time_value)  # 小端序，无符号短整型
        return list(time_bytes)
    
    def calculate_checksum(self, data_bytes):
        """
        计算校验码：所有数据字节相加取反
        如果相加结果超过255，则取最低字节再取反
        :param data_bytes: 数据字节列表
        :return: 校验码
        """
        total = sum(data_bytes)
        # 取最低字节
        low_byte = total & 0xFF
        # 取反
        checksum = (~low_byte) & 0xFF
        return checksum
    
    def read_current(self):
        """
        读取舵机电流
        命令: FF FF 0A 04 02 45 02 A8
        返回数据: 最后一字节为校验位，倒数第二和第三字节为电流数(高八位在后，低八位在前)
        转换: 10进制 * 6.5 / 1000 = 电流值(A)
        :return: 电流值(A)，失败返回None
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        
        try:
            # 清空输入缓冲区，确保读取的是新数据
            self.serial_conn.reset_input_buffer()
            
            # 发送读取电流命令
            current_cmd = [0xFF, 0xFF, 0x0A, 0x04, 0x02, 0x45, 0x02, 0xA8]
            self.serial_conn.write(bytes(current_cmd))
            self.serial_conn.flush()
            
            # 等待一小段时间确保响应到达
            time.sleep(0.05)
            
            # 读取响应 (假设返回8字节数据)
            response = self.serial_conn.read(8)
            
            # 验证响应是否正确（检查帧头）
            if len(response) >= 3 and response[0] == 0xFF and response[1] == 0xFF:
                # 倒数第三字节(低八位) 和 倒数第二字节(高八位)
                low_byte = response[-3]
                high_byte = response[-2]
                
                # 组合为16位数值 (高八位在后，低八位在前)
                current_raw = low_byte + (high_byte << 8)
                
                # 转换为电流值
                current_value = current_raw * 6.5 / 1000.0

                return current_value
            else:
                print(f"读取电流响应格式错误，响应数据: {response.hex()}")
                return None
                
        except Exception as e:
            print(f"读取电流失败: {e}")
            return None
    
    def read_position(self):
        """
        读取舵机位置
        命令: FF FF 0A 04 02 38 02 B5
        返回数据: 最后一字节为校验位，倒数第二和第三字节为位置数(高八位在后，低八位在前)
        :return: 位置值，失败返回None
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        
        try:
            # 清空输入缓冲区，确保读取的是新数据
            self.serial_conn.reset_input_buffer()
            
            # 发送读取位置命令
            position_cmd = [0xFF, 0xFF, 0x0A, 0x04, 0x02, 0x38, 0x02, 0xB5]
            self.serial_conn.write(bytes(position_cmd))
            self.serial_conn.flush()
            
            # 等待一小段时间确保响应到达
            time.sleep(0.05)
            
            # 读取响应 (假设返回8字节数据)
            response = self.serial_conn.read(8)
            
            # 验证响应是否正确（检查帧头）
            if len(response) >= 3 and response[0] == 0xFF and response[1] == 0xFF:
                # 倒数第三字节(低八位) 和 倒数第二字节(高八位)
                low_byte = response[-3]
                high_byte = response[-2]
                
                # 组合为16位数值 (高八位在后，低八位在前)
                position_value = low_byte + (high_byte << 8)
                
                return position_value
            else:
                print(f"读取位置响应格式错误，响应数据: {response.hex()}")
                return None
                
        except Exception as e:
            print(f"读取位置失败: {e}")
            return None
    
    def monitor_servo(self):
        """
        监视舵机电流和位置的线程函数
        当电流大于阈值时，发送停止命令(设置目标位置为当前位置)
        当位置稳定时自动停止监视
        """
        print(f"开始监视舵机，电流阈值: {self.current_threshold}A")
        
        previous_position = None
        stable_count = 0
        required_stable_count = 3  # 需要连续3次位置稳定才认为到达目标
        position_tolerance = 5  # 位置变化容差
        
        while self.monitoring:
            try:
                # 读取当前电流
                current = self.read_current()
                if current is not None:
                    print(f"当前电流: {current:.3f}A\n")
                    
                    # 检查电流是否超过阈值
                    if current > self.current_threshold:
                        print(f" 当前电流: {current:.3f}A - 电流超过阈值({self.current_threshold}A)!")
                        
                        # 读取当前位置
                        current_position = self.read_position()
                        if current_position is not None:
                            print(f"发送停止命令，当前位置: {current_position}")
                            
                            # 发送命令让电机停在当前位置
                            if self.last_speed_command is not None:
                                # 使用较低的速度发送当前位置命令
                                self.send_command(current_position, min(self.last_speed_command, 100))
                            else:
                                self.send_command(current_position, 100)  # 默认低速
                            
                            # 电流超阈值后停止监视
                            print("电流超阈值，停止监视")
                            self.monitoring = False
                            break
                
                # 读取当前位置用于显示和稳定性检查
                position = self.read_position()
                if position is not None:
                    # print(f"当前位置: {position}")
                    
                    # 检查位置稳定性
                    if previous_position is not None:
                        position_change = abs(position - previous_position)
                        if position_change <= position_tolerance:
                            stable_count += 1
                            print(f"位置稳定，计数: {stable_count}/{required_stable_count}")
                            if stable_count >= required_stable_count:
                                print("位置已稳定，停止监视")
                                self.monitoring = False
                                break
                        else:
                            stable_count = 0  # 重置稳定计数
                    
                    previous_position = position
                
                # 等待下次检测
                time.sleep(self.monitor_frequency)
                
            except Exception as e:
                print(f"监视过程出错: {e}")
                break
        
        print("监视线程结束")
    
    def start_monitoring(self):
        """
        开始监视舵机状态
        """
        if self.monitoring:
            print("监视已在运行中")
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self.monitor_servo, daemon=True)
        self.monitor_thread.start()
    
    def stop_monitoring(self):
        """
        停止监视舵机状态
        """
        self.monitoring = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        print("监视已停止")
    
    def send_command_with_monitoring_percent(self, position_percent, speed_percent, time_value=0, wait_for_completion=True):
        """
        使用百分比发送命令并开始监视
        :param position_percent: 位置百分比 (0-100)
        :param speed_percent: 速度百分比 (0-100)
        :param time_value: 时间值，默认为0
        :param wait_for_completion: 是否等待运动完成
        :return: 是否发送成功
        """
        # 先发送命令
        success = self.send_command_percent(position_percent, speed_percent, time_value)
        
        if success:
            # 等待控制命令处理完成，并清空串口缓冲区
            time.sleep(0.1)
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.reset_input_buffer()  # 清空输入缓冲区
                print("已清空串口输入缓冲区")
            
            # 开始监视
            self.start_monitoring()
            
            if wait_for_completion:
                # 等待监视线程完成
                while self.monitoring and self.monitor_thread and self.monitor_thread.is_alive():
                    time.sleep(0.1)
        
        return success
    
    def send_command_with_monitoring(self, position, speed, time_value=0, wait_for_completion=True):
        """
        发送命令并开始监视
        :param position: 位置值 (1600-3200)
        :param speed: 速度值 (0-1000)
        :param time_value: 时间值，默认为0
        :param wait_for_completion: 是否等待运动完成
        :return: 是否发送成功
        """
        # 先发送命令
        success = self.send_command(position, speed, time_value)
        
        if success:
            # 等待控制命令处理完成，并清空串口缓冲区
            time.sleep(0.1)
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.reset_input_buffer()  # 清空输入缓冲区
                print("已清空串口输入缓冲区")
            
            # 开始监视
            self.start_monitoring()
            
            if wait_for_completion:
                # 等待监视线程完成
                while self.monitoring and self.monitor_thread and self.monitor_thread.is_alive():
                    time.sleep(0.1)
        
        return success
    
    def create_message_percent(self, position_percent, speed_percent, time_value=0):
        """
        使用百分比创建完整的消息
        :param position_percent: 位置百分比 (0-100)
        :param speed_percent: 速度百分比 (0-100)
        :param time_value: 时间值，默认为0
        :return: 完整消息的字节列表
        """
        # 将百分比转换为实际值
        position = self.position_percent_to_value(position_percent)
        speed = self.speed_percent_to_value(speed_percent)
        
        # 调用原有的创建消息方法
        return self.create_message(position, speed, time_value)
    
    def create_message(self, position, speed, time_value=0):
        """
        创建完整的消息
        :param position: 位置值 (1600-3200)
        :param speed: 速度值 (0-1000)
        :param time_value: 时间值，默认为0
        :return: 完整消息的字节列表
        """
        # 完整消息从帧头开始
        message = self.frame_header.copy()
        
        # 需要计算校验码的数据部分（命令头 + 位置 + 时间 + 速度）
        checksum_data = self.command_header.copy()
        
        # 添加位置字节
        position_bytes = self.position_to_hex(position)
        checksum_data.extend(position_bytes)
        
        # 添加时间字节
        time_bytes = self.time_to_hex(time_value)
        checksum_data.extend(time_bytes)
        
        # 添加速度字节
        speed_bytes = self.speed_to_hex(speed)
        checksum_data.extend(speed_bytes)
        
        # 将命令头、位置、时间、速度添加到完整消息中
        message.extend(checksum_data)
        
        # 计算校验码（只对checksum_data计算）
        checksum = self.calculate_checksum(checksum_data)
        message.append(checksum)
        
        return message
    
    def send_command_percent(self, position_percent, speed_percent, time_value=0):
        """
        使用百分比发送命令到夹爪
        :param position_percent: 位置百分比 (0-100)
        :param speed_percent: 速度百分比 (0-100)
        :param time_value: 时间值，默认为0
        :return: 是否发送成功
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("串口未连接")
            return False
        
        try:
            # 将百分比转换为实际值
            position = self.position_percent_to_value(position_percent)
            speed = self.speed_percent_to_value(speed_percent)
            
            # 创建消息
            message = self.create_message(position, speed, time_value)
            
            # 转换为字节数组
            message_bytes = bytes(message)
            
            # 发送消息
            self.serial_conn.write(message_bytes)
            self.serial_conn.flush()
            
            # 记录最后发送的命令
            self.last_position_command = position
            self.last_speed_command = speed
            
            # 打印发送的消息
            hex_string = ' '.join([f'{b:02X}' for b in message])
            print(f"发送消息: {hex_string}")
            print(f"位置: {position_percent}% (实际值: {position}), 时间: {time_value}, 速度: {speed_percent}% (实际值: {speed})")
            
            return True
            
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False
    
    def send_command(self, position, speed, time_value=0):
        """
        发送命令到夹爪
        :param position: 位置值 (1600-3200)
        :param speed: 速度值 (0-1000)
        :param time_value: 时间值，默认为0
        :return: 是否发送成功
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("串口未连接")
            return False
        
        try:
            # 创建消息
            message = self.create_message(position, speed, time_value)
            
            # 转换为字节数组
            message_bytes = bytes(message)
            
            # 发送消息
            self.serial_conn.write(message_bytes)
            self.serial_conn.flush()
            
            # 记录最后发送的命令
            self.last_position_command = position
            self.last_speed_command = speed
            
            # 打印发送的消息
            hex_string = ' '.join([f'{b:02X}' for b in message])
            print(f"发送消息: {hex_string}")
            print(f"位置: {position}, 时间: {time_value}, 速度: {speed}")
            
            return True
            
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False
    
    def print_message_format_percent(self, position_percent, speed_percent, time_value=0):
        """
        使用百分比打印消息格式（用于调试）
        """
        # 将百分比转换为实际值
        position = self.position_percent_to_value(position_percent)
        speed = self.speed_percent_to_value(speed_percent)
        
        message = self.create_message(position, speed, time_value)
        
        print("消息格式分解:")
        print(f"帧头 (不参与校验): {' '.join([f'{b:02X}' for b in self.frame_header])}")
        print(f"命令头 (参与校验): {' '.join([f'{b:02X}' for b in self.command_header])}")
        
        position_bytes = self.position_to_hex(position)
        print(f"位置 ({position_percent}% -> {position}): {' '.join([f'{b:02X}' for b in position_bytes])}")
        
        time_bytes = self.time_to_hex(time_value)
        print(f"时间 ({time_value}): {' '.join([f'{b:02X}' for b in time_bytes])}")
        
        speed_bytes = self.speed_to_hex(speed)
        print(f"速度 ({speed_percent}% -> {speed}): {' '.join([f'{b:02X}' for b in speed_bytes])}")
        
        # 计算校验码的数据部分
        checksum_data = self.command_header.copy()
        checksum_data.extend(position_bytes)
        checksum_data.extend(time_bytes)
        checksum_data.extend(speed_bytes)
        checksum = self.calculate_checksum(checksum_data)
        print(f"校验码: {checksum:02X}")
        
        hex_string = ' '.join([f'{b:02X}' for b in message])
        print(f"完整消息: {hex_string}")

    def print_message_format(self, position, speed, time_value=0):
        """
        打印消息格式（用于调试）
        """
        message = self.create_message(position, speed, time_value)
        
        print("消息格式分解:")
        print(f"帧头 (不参与校验): {' '.join([f'{b:02X}' for b in self.frame_header])}")
        print(f"命令头 (参与校验): {' '.join([f'{b:02X}' for b in self.command_header])}")
        
        position_bytes = self.position_to_hex(position)
        print(f"位置 ({position}): {' '.join([f'{b:02X}' for b in position_bytes])}")
        
        time_bytes = self.time_to_hex(time_value)
        print(f"时间 ({time_value}): {' '.join([f'{b:02X}' for b in time_bytes])}")
        
        speed_bytes = self.speed_to_hex(speed)
        print(f"速度 ({speed}): {' '.join([f'{b:02X}' for b in speed_bytes])}")
        
        # 计算校验码的数据部分
        checksum_data = self.command_header.copy()
        checksum_data.extend(position_bytes)
        checksum_data.extend(time_bytes)
        checksum_data.extend(speed_bytes)
        checksum = self.calculate_checksum(checksum_data)
        print(f"校验码: {checksum:02X}")
        
        hex_string = ' '.join([f'{b:02X}' for b in message])
        print(f"完整消息: {hex_string}")


def main():
    """主函数"""
    # 创建夹爪控制器实例
    gripper = GripperController(port='/dev/ttyUSB1', baudrate=115200)
    
    try:
        # # 示例：不连接串口，仅显示消息格式（使用百分比）
        # print("=== 夹爪控制消息格式示例（百分比输入） ===")
        # # 示例3：位置100%，速度100%
        # print("\n示例3:")
        # gripper.print_message_format_percent(100, 100)
        

        # 连接串口
        if gripper.connect():
            print("\n=== 演示监视功能 ===")
            
            # 方法1: 发送命令并等待运动完成（通过电流阈值或位置稳定自动终止）
            print("\n1. 发送命令并等待运动完成:")
            gripper.send_command_with_monitoring_percent(50, 50, wait_for_completion=True)
            
            # # 方法2: 手动控制监视
            # print("\n2. 手动控制监视:")
            # gripper.send_command_percent(0, 30)  # 发送命令
            # gripper.start_monitoring()  # 开始监视
            
            # # 等待3秒或监视自动结束
            # start_time = time.time()
            # while gripper.monitoring and (time.time() - start_time) < 3:
            #     time.sleep(0.1)
            
            # if gripper.monitoring:
            #     gripper.stop_monitoring()   # 如果还在监视则手动停止
            
            # # 方法3: 仅测试读取电流和位置
            # print("\n3. 测试读取功能:")
            # current = gripper.read_current()
            # position = gripper.read_position()
            # if current is not None:
            #     print(f"当前电流: {current:.3f}A")
            # if position is not None:
            #     print(f"当前位置: {position}")
            
            print("\n测试完成!")
    
    except Exception as e:
        print(f"程序错误: {e}")
    
    finally:
        gripper.stop_monitoring()  # 确保停止监视
        gripper.disconnect()
        print("程序结束")


if __name__ == "__main__":
    main()