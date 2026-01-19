import struct
import time
from typing import Optional, Tuple, List
import serial
import crcmod


class CL57RDriver:
    """
    CL57-R 系列步进驱动器 Modbus RTU 控制类
    基于用户手册 V1.0 (57R系列-R..7）.pdf 实现
    """
    
    # 功能码
    FUNC_READ = 0x03
    FUNC_WRITE_SINGLE = 0x06
    FUNC_WRITE_MULTIPLE = 0x10
    
    # 常用寄存器地址（十进制）
    REG_STATUS = 4           # 运行状态
    REG_CURRENT_POS_H = 8   # 当前位置高16位
    REG_CURRENT_POS_L = 9   # 当前位置低16位
    REG_CURRENT_SPEED = 10  # 当前速度 (r/min)
    REG_CONTROL_WORD = 78   # 控制字 (0x004E)
    REG_TARGET_POS_H = 55   # 定位目标高16位
    REG_TARGET_POS_L = 56   # 定位目标低16位
    REG_POS_SPEED = 54      # 定位运行速度
    REG_JOG_SPEED = 48      # JOG运行速度 (PA_030)
    REG_JOG_ACC_TIME = 49   # JOG加速时间 (PA_031)
    REG_JOG_DEC_TIME = 50   # JOG减速时间 (PA_032)
    REG_SUBDIVISION = 35    # 细分设置
    
    def __init__(self, port: str, baudrate: int = 38400, slave_id: int = 1, timeout: float = 0.1):
        """
        初始化驱动器连接
        
        Args:
            port: 串口端口，如 'COM3' 或 '/dev/ttyUSB0'
            baudrate: 波特率，支持9600, 19200, 38400, 115200
            slave_id: 从站ID (1-31)
            timeout: 串口超时时间
        """
        self.slave_id = slave_id
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=1,
            timeout=timeout
        )
        
        # 创建CRC16校验函数
        self.crc16 = crcmod.predefined.mkCrcFun('modbus')
        
        # 当前位置和速度缓存
        self._current_position = 0
        self._current_speed = 0
        self._last_update_time = time.time()
        
    def _calculate_crc(self, data: bytes) -> bytes:
        """计算Modbus CRC16校验码"""
        crc_value = self.crc16(data)
        # 低位在前，高位在后
        return struct.pack('<H', crc_value)
    
    def _send_receive(self, data: bytes, expected_length: Optional[int] = None) -> bytes:
        """
        发送数据并接收响应
        
        Args:
            data: 要发送的数据
            expected_length: 期望的响应长度
            
        Returns:
            响应数据
        """
        # 清空输入缓冲区
        self.serial.reset_input_buffer()
        
        # 发送数据
        self.serial.write(data)
        
        # 等待响应
        time.sleep(0.01)  # 最小响应时间
        
        if expected_length:
            response = self.serial.read(expected_length)
        else:
            # 读取所有可用数据
            response = self.serial.read_all()
            
        return response
    
    def _read_registers(self, start_addr: int, count: int) -> List[int]:
        """
        读取多个寄存器
        
        Args:
            start_addr: 起始地址
            count: 寄存器数量
            
        Returns:
            寄存器值列表
        """
        # 构建读命令
        cmd = struct.pack('>BBHH', 
                         self.slave_id, 
                         self.FUNC_READ, 
                         start_addr, 
                         count)
        cmd += self._calculate_crc(cmd)
        
        # 发送并接收
        response = self._send_receive(cmd, 5 + 2 * count)
        
        if len(response) < 5:
            raise Exception(f"响应长度不足: {len(response)} bytes")
            
        # 验证响应
        if response[0] != self.slave_id or response[1] != self.FUNC_READ:
            raise Exception("响应格式错误")
            
        # 检查错误响应
        if response[1] & 0x80:
            error_code = response[2]
            raise Exception(f"Modbus错误码: {error_code:02X}")
            
        data_length = response[2]
        if data_length != count * 2:
            raise Exception(f"数据长度不匹配: 期望{count*2}, 收到{data_length}")
            
        # 解析数据
        values = []
        for i in range(count):
            idx = 3 + i * 2
            value = struct.unpack('>H', response[idx:idx+2])[0]
            values.append(value)
            
        return values
    
    def _write_single_register(self, addr: int, value: int) -> bool:
        """
        写入单个寄存器
        
        Args:
            addr: 寄存器地址
            value: 要写入的值
            
        Returns:
            是否成功
        """
        # 构建写命令
        cmd = struct.pack('>BBHH', 
                         self.slave_id, 
                         self.FUNC_WRITE_SINGLE, 
                         addr, 
                         value)
        cmd += self._calculate_crc(cmd)
        
        # 发送并接收
        response = self._send_receive(cmd, 8)
        
        if len(response) < 8:
            return False
            
        # 验证响应（响应应和请求相同）
        expected = struct.pack('>BBHH', 
                              self.slave_id, 
                              self.FUNC_WRITE_SINGLE, 
                              addr, 
                              value)
        expected += self._calculate_crc(expected)
        
        return response == expected
    
    def _write_multiple_registers(self, start_addr: int, values: List[int]) -> bool:
        """
        写入多个寄存器
        
        Args:
            start_addr: 起始地址
            values: 要写入的值列表
            
        Returns:
            是否成功
        """
        count = len(values)
        byte_count = count * 2
        
        # 构建写命令
        header = struct.pack('>BBHHB', 
                            self.slave_id, 
                            self.FUNC_WRITE_MULTIPLE, 
                            start_addr, 
                            count, 
                            byte_count)
        
        # 添加数据
        data = header
        for value in values:
            data += struct.pack('>H', value)
            
        # 添加CRC
        data += self._calculate_crc(data)
        
        # 发送并接收
        response = self._send_receive(data, 8)
        
        if len(response) < 8:
            return False
            
        # 验证响应（返回起始地址和数量）
        expected = struct.pack('>BBHH', 
                              self.slave_id, 
                              self.FUNC_WRITE_MULTIPLE, 
                              start_addr, 
                              count)
        expected += self._calculate_crc(expected)
        
        return response == expected
    
    def get_position(self) -> int:
        """
        读取当前位置
        
        Returns:
            当前位置（脉冲数）
        """
        try:
            # 读取高16位和低16位
            pos_h = self._read_registers(self.REG_CURRENT_POS_H, 1)[0]
            pos_l = self._read_registers(self.REG_CURRENT_POS_L, 1)[0]
            
            # 组合成32位有符号整数
            # 注意：需要根据实际的数据格式调整，文档中位置是INTEGER32
            position = (pos_h << 16) | pos_l
            if position & 0x80000000:  # 处理负数
                position = -((~position + 1) & 0xFFFFFFFF)
                
            self._current_position = position
            self._last_update_time = time.time()
            return position
            
        except Exception as e:
            print(f"读取位置失败: {e}")
            return self._current_position
    
    def get_speed(self) -> int:
        """
        读取当前速度
        
        Returns:
            当前速度 (r/min)
        """
        try:
            # 当前速度寄存器是INTEGER16
            speed = self._read_registers(self.REG_CURRENT_SPEED, 1)[0]
            # 处理有符号数
            if speed & 0x8000:
                speed = -((~speed + 1) & 0xFFFF)
                
            self._current_speed = speed
            return speed
            
        except Exception as e:
            print(f"读取速度失败: {e}")
            return self._current_speed
    
    def get_status(self) -> dict:
        """
        读取驱动器状态
        
        Returns:
            状态字典，包含到位、回零完成、电机运行、故障等状态位
        """
        try:
            status_value = self._read_registers(self.REG_STATUS, 1)[0]
            
            status = {
                '到位': bool(status_value & 0x01),
                '回零完成': bool(status_value & 0x02),
                '电机运行': bool(status_value & 0x04),
                '故障': bool(status_value & 0x08),
                '电机使能': bool(status_value & 0x10),
                '正软限位': bool(status_value & 0x20),
                '负软限位': bool(status_value & 0x40),
                '原始值': status_value
            }
            return status
            
        except Exception as e:
            print(f"读取状态失败: {e}")
            return {}
    
    def set_position_mode(self, target_position: int, speed: int = 100, 
                         is_absolute: bool = True, start_speed: int = 0,
                         acc_time: int = 100, dec_time: int = 100) -> bool:
        """
        设置位置模式运行
        
        Args:
            target_position: 目标位置（脉冲数）
            speed: 运行速度 (r/min，0-3000)
            is_absolute: True为绝对位置，False为相对位置
            start_speed: 起始速度 (r/min)
            acc_time: 加速时间 (ms)
            dec_time: 减速时间 (ms)
            
        Returns:
            是否成功
        """
        try:
            # 1. 设置运行参数
            self._write_single_register(51, start_speed)  # PA_033: 起始速度
            self._write_single_register(52, acc_time)     # PA_034: 加速时间
            self._write_single_register(53, dec_time)     # PA_035: 减速时间
            self._write_single_register(54, speed)        # PA_036: 运行速度
            
            # 2. 设置目标位置（32位）
            # 将32位有符号整数拆分为两个16位
            target_32 = target_position & 0xFFFFFFFF
            target_h = (target_32 >> 16) & 0xFFFF
            target_l = target_32 & 0xFFFF
            
            self._write_single_register(55, target_h)     # PA_037: 目标高16位
            self._write_single_register(56, target_l)     # PA_038: 目标低16位
            
            # 3. 设置控制字
            control_word = 0
            control_word |= 0x01  # Bit0: 定位控制有效
            if is_absolute:
                control_word |= 0x02  # Bit1: 绝对位置模式
            control_word |= 0x04  # Bit2: 允许打断当前运动
            
            return self._write_single_register(self.REG_CONTROL_WORD, control_word)
            
        except Exception as e:
            print(f"设置位置模式失败: {e}")
            return False
    
    def jog_forward(self, speed: int = 100, acc_time: int = 100, dec_time: int = 100) -> bool:
        """
        正向JOG运行 - 修正版
        
        Args:
            speed: 速度 (r/min，正数)
            acc_time: 加速时间 (ms)
            dec_time: 减速时间 (ms)
            
        Returns:
            是否成功
        """
        try:
            # 1. 设置JOG参数
            # 写入JOG速度
            if speed < 0:
                speed = -speed  # 确保为正值
            
            self._write_single_register(self.REG_JOG_SPEED, speed)
            self._write_single_register(self.REG_JOG_ACC_TIME, acc_time)
            self._write_single_register(self.REG_JOG_DEC_TIME, dec_time)
            
            # 2. 设置控制字，只设置Bit3为1，其他位为0
            control_word = 0x08  # Bit3: JOG控制有效
            
            return self._write_single_register(self.REG_CONTROL_WORD, control_word)
            
        except Exception as e:
            print(f"正向JOG失败: {e}")
            return False

    def jog_reverse(self, speed: int = 100, acc_time: int = 100, dec_time: int = 100) -> bool:
        """
        反向JOG运行 - 修正版
        
        Args:
            speed: 速度 (r/min，取绝对值)
            acc_time: 加速时间 (ms)
            dec_time: 减速时间 (ms)
            
        Returns:
            是否成功
        """
        try:
            # 1. 设置JOG参数
            # 反向JOG速度需要设置为负值
            speed = -abs(speed)
            
            # 转换为16位补码
            if speed < 0:
                speed_value = (~(-speed) + 1) & 0xFFFF
            else:
                speed_value = speed
            
            self._write_single_register(self.REG_JOG_SPEED, speed_value)
            self._write_single_register(self.REG_JOG_ACC_TIME, acc_time)
            self._write_single_register(self.REG_JOG_DEC_TIME, dec_time)
            
            # 2. 设置控制字，只设置Bit3为1，其他位为0
            control_word = 0x08  # Bit3: JOG控制有效
            
            return self._write_single_register(self.REG_CONTROL_WORD, control_word)
            
        except Exception as e:
            print(f"反向JOG失败: {e}")
            return False
    
    def stop(self) -> bool:
        """
        停止运动
        
        Returns:
            是否成功
        """
        try:
            # 设置控制字Bit5为1
            control_word = self._read_registers(self.REG_CONTROL_WORD, 1)[0]
            control_word |= 0x20  # Bit5: 停止控制
            
            return self._write_single_register(self.REG_CONTROL_WORD, control_word)
            
        except Exception as e:
            print(f"停止失败: {e}")
            return False
    
    def emergency_stop(self) -> bool:
        """
        急停
        
        Returns:
            是否成功
        """
        try:
            # 设置控制字Bit6为1
            control_word = self._read_registers(self.REG_CONTROL_WORD, 1)[0]
            control_word |= 0x40  # Bit6: 急停控制
            
            return self._write_single_register(self.REG_CONTROL_WORD, control_word)
            
        except Exception as e:
            print(f"急停失败: {e}")
            return False
    
    def enable_motor(self, enable: bool = True) -> bool:
        """
        电机使能/释放
        
        Args:
            enable: True为使能，False为释放
            
        Returns:
            是否成功
        """
        try:
            # 辅助控制字PA_04F
            value = 0x0500 if enable else 0x0600  # 0x0500: 电机使能, 0x0600: 电机释放
            return self._write_single_register(79, value)  # PA_04F
            
        except Exception as e:
            print(f"电机使能控制失败: {e}")
            return False
    
    def set_subdivision(self, steps_per_rev: int = 1000) -> bool:
        """
        设置细分
        
        Args:
            steps_per_rev: 每转脉冲数 (400-51200)
            
        Returns:
            是否成功
        """
        try:
            if steps_per_rev < 400 or steps_per_rev > 51200:
                print(f"细分值 {steps_per_rev} 超出范围 400-51200")
                return False
                
            return self._write_single_register(self.REG_SUBDIVISION, steps_per_rev)
            
        except Exception as e:
            print(f"设置细分失败: {e}")
            return False
    
    def home(self, home_mode: int = 24, home_speed: int = 100, 
             crawl_speed: int = 10, acc_time: int = 100) -> bool:
        """
        回零操作
        
        Args:
            home_mode: 回零方式 (17:负限位, 18:正限位, 24:正向原点, 29:反向原点, 35:当前位置)
            home_speed: 回零速度 (r/min)
            crawl_speed: 爬行速度 (r/min)
            acc_time: 加减速时间 (ms)
            
        Returns:
            是否成功
        """
        try:
            # 设置回零参数
            self._write_single_register(64, home_mode)      # PA_040: 回零方式
            self._write_single_register(65, home_speed)     # PA_041: 回零速度
            self._write_single_register(66, crawl_speed)    # PA_042: 爬行速度
            self._write_single_register(67, acc_time)       # PA_043: 加减速时间
            
            # 设置控制字Bit4为1触发回零
            control_word = self._read_registers(self.REG_CONTROL_WORD, 1)[0]
            control_word |= 0x10  # Bit4: 回零控制
            
            return self._write_single_register(self.REG_CONTROL_WORD, control_word)
            
        except Exception as e:
            print(f"回零操作失败: {e}")
            return False
    
    def save_parameters(self) -> bool:
        """保存当前参数到EEPROM"""
        try:
            return self._write_single_register(79, 0x0200)  # PA_04F: 保存参数
        except Exception as e:
            print(f"保存参数失败: {e}")
            return False
    
    def restore_factory_parameters(self) -> bool:
        """恢复出厂参数"""
        try:
            return self._write_single_register(79, 0x0100)  # PA_04F: 恢复出厂参数
        except Exception as e:
            print(f"恢复出厂参数失败: {e}")
            return False
    
    def close(self):
        """关闭串口连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()


# 使用示例
if __name__ == "__main__":
    # 创建驱动器实例（请根据实际情况修改端口和ID）
    driver = CL57RDriver(port='/dev/ttyUSB0', baudrate=38400, slave_id=1)
    
    try:
        # 1. 读取当前状态
        position = driver.get_position()
        speed = driver.get_speed()
        status = driver.get_status()
        
        print(f"当前位置: {position} 脉冲")
        print(f"当前速度: {speed} r/min")
        print(f"状态: {status}")
        
        # 2. 设置细分（每转脉冲数）
        driver.set_subdivision(1000)  # 1000脉冲/转
        
        # 3. 电机使能
        driver.enable_motor(True)
        
        # 4. 相对位置移动
        # print("执行相对位置移动: +5000脉冲，速度100r/min")
        # driver.set_position_mode(
        #     target_position=5000,
        #     speed=100,
        #     is_absolute=False
        # )
        
        # # 等待运动完成
        # time.sleep(3)
        
        # 5. 绝对位置移动
        driver.set_position_mode(
            target_position=10000,
            speed=1000,
            is_absolute=True
        )
        
        # time.sleep(3)
        
        # 6. 点动运行
        # driver.jog_forward(speed=200, acc_time=100, dec_time=100)
        # time.sleep(10)
        # driver.stop()
        
        # driver.jog_reverse(speed=1000, acc_time=100, dec_time=100)
        # time.sleep(10)
        # driver.stop()
        
        # # 7. 回零操作
        # print("执行回零操作")
        # driver.home(home_mode=24, home_speed=100, crawl_speed=10)
        
        # # 等待回零完成
        # while True:
        #     status = driver.get_status()
        #     if status.get('回零完成'):
        #         print("回零完成")
        #         break
        #     time.sleep(0.1)
        
        # 8. 保存参数
        # driver.save_parameters()
        
    finally:
        # 关闭连接
        driver.close()
        print("程序结束")