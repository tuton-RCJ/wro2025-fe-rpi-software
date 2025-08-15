#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
BNO055 9軸フュージョンセンサテストプログラム
UART通信版 (/dev/ttyUSB0)
参考: https://rott1st.hatenablog.com/entry/2022/06/02/112232
"""

import serial
import struct
import time


class BNO055:
    """BNO055 UARTクラス"""
    
    # BNO055のID
    BNO055_ID = 0xA0
    
    # Power mode settings
    POWER_MODE_NORMAL = 0X00
    POWER_MODE_LOWPOWER = 0X01
    POWER_MODE_SUSPEND = 0X02
    
    # Operation mode settings
    OPERATION_MODE_CONFIG = 0X00
    OPERATION_MODE_ACCONLY = 0X01
    OPERATION_MODE_MAGONLY = 0X02
    OPERATION_MODE_GYRONLY = 0X03
    OPERATION_MODE_ACCMAG = 0X04
    OPERATION_MODE_ACCGYRO = 0X05
    OPERATION_MODE_MAGGYRO = 0X06
    OPERATION_MODE_AMG = 0X07
    OPERATION_MODE_IMUPLUS = 0X08
    OPERATION_MODE_COMPASS = 0X09
    OPERATION_MODE_M4G = 0X0A
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B
    OPERATION_MODE_NDOF = 0X0C
    
    # Output vector type
    VECTOR_ACCELEROMETER = 0x08
    VECTOR_MAGNETOMETER = 0x0E
    VECTOR_GYROSCOPE = 0x14
    VECTOR_EULER = 0x1A
    VECTOR_LINEARACCEL = 0x28
    VECTOR_GRAVITY = 0x2E
    
    # レジスタアドレス定義
    BNO055_CHIP_ID_ADDR = 0x00
    BNO055_ACCEL_REV_ID_ADDR = 0x01
    BNO055_MAG_REV_ID_ADDR = 0x02
    BNO055_GYRO_REV_ID_ADDR = 0x03
    BNO055_SW_REV_ID_LSB_ADDR = 0x04
    BNO055_SW_REV_ID_MSB_ADDR = 0x05
    BNO055_BL_REV_ID_ADDR = 0X06
    BNO055_PAGE_ID_ADDR = 0X07
    
    # データレジスタ
    BNO055_EULER_H_LSB_ADDR = 0X1A
    BNO055_EULER_H_MSB_ADDR = 0X1B
    BNO055_EULER_R_LSB_ADDR = 0X1C
    BNO055_EULER_R_MSB_ADDR = 0X1D
    BNO055_EULER_P_LSB_ADDR = 0X1E
    BNO055_EULER_P_MSB_ADDR = 0X1F
    
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20
    BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21
    BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22
    BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24
    BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26
    BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27
    
    # ステータスレジスタ
    BNO055_CALIB_STAT_ADDR = 0X35
    BNO055_SELFTEST_RESULT_ADDR = 0X36
    BNO055_SYS_STAT_ADDR = 0X39
    BNO055_SYS_ERR_ADDR = 0X3A
    BNO055_TEMP_ADDR = 0X34
    
    # コントロールレジスタ
    BNO055_OPR_MODE_ADDR = 0X3D
    BNO055_PWR_MODE_ADDR = 0X3E
    BNO055_SYS_TRIGGER_ADDR = 0X3F
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        """コンストラクタ"""
        self.port = port
        self.baudrate = baudrate
        self.uart = None
        self._mode = self.OPERATION_MODE_NDOF
    
    def begin(self, mode=None):
        """BNO055を初期化"""
        if mode is None:
            mode = self.OPERATION_MODE_NDOF
        
        try:
            # シリアルポートを開く
            self.uart = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"シリアルポート {self.port} を開きました (ボーレート: {self.baudrate})")
            time.sleep(0.5)  # 起動時間を長めにとる
            
            # チップIDを確認
            print("BNO055のチップIDを確認中...")
            chip_id = self.read_register(self.BNO055_CHIP_ID_ADDR)
            if chip_id is None:
                print("BNO055からの応答がありません。デバイスが接続されているかUARTが有効か確認してください。")
                return False
            
            if chip_id != self.BNO055_ID:
                print(f"チップID不一致: 0x{chip_id:02X} (期待値: 0x{self.BNO055_ID:02X})")
                time.sleep(1)  # デバイスの起動を待つ
                chip_id = self.read_register(self.BNO055_CHIP_ID_ADDR)
                if chip_id is None:
                    print("BNO055からの応答がありません（再試行後）。")
                    return False
                if chip_id != self.BNO055_ID:
                    print(f"BNO055が見つかりません。チップID: 0x{chip_id:02X} (期待値: 0x{self.BNO055_ID:02X})")
                    return False
            
            print(f"BNO055が見つかりました。チップID: 0x{chip_id:02X}")
            
            # 動作モードを直接設定（リセットせずに）
            print(f"動作モードを設定中: 0x{mode:02X}")
            if not self.write_register(self.BNO055_OPR_MODE_ADDR, mode):
                print("動作モードの設定に失敗しました")
                return False
            
            self._mode = mode
            time.sleep(0.1)  # モード切り替え時間
            
            # モードが正しく設定されたか確認
            current_mode = self.read_register(self.BNO055_OPR_MODE_ADDR)
            if current_mode != mode:
                print(f"モード設定エラー: 現在のモード 0x{current_mode:02X} (期待値: 0x{mode:02X})")
                return False
            
            print("BNO055の初期化が完了しました")
            return True
            
        except Exception as e:
            print(f"BNO055の初期化エラー: {e}")
            return False
    
    def set_mode(self, mode):
        """動作モードを設定"""
        self._mode = mode
        self.write_register(self.BNO055_OPR_MODE_ADDR, self._mode)
        time.sleep(0.03)
    
    def read_register(self, register, length=1):
        """レジスタを読み取り"""
        if self.uart is None:
            return None
            
        try:
            # バッファをクリア
            self.uart.flushInput()
            
            # BNO055 UARTプロトコル（0xAA, 0x01, register, length）
            cmd = bytes([0xAA, 0x01, register, length])
            self.uart.write(cmd)
            time.sleep(0.01)
            
            # レスポンスを読み取り
            resp = self.uart.read(length + 2)
            
            # デバッグ情報
            if len(resp) == 0:
                print(f"応答なし: レジスタ0x{register:02X}")
                return None
            
            if len(resp) < 2:
                print(f"応答が短すぎます: {len(resp)}バイト, データ: {resp.hex()}")
                return None
            
            if resp[0] != 0xBB:
                print(f"無効なレスポンスヘッダ: 0x{resp[0]:02X} (期待値: 0xBB)")
                return None
            
            if len(resp) < length + 2:
                print(f"データが不足: 受信{len(resp)}バイト, 期待{length + 2}バイト")
                return None
            
            # データを返す
            if length == 1:
                return int(resp[2])
            else:
                return resp[2:2+length]
                
        except Exception as e:
            print(f"レジスタ読み取りエラー (0x{register:02X}): {e}")
            return None
    
    def write_register(self, register, value):
        """レジスタに書き込み"""
        if self.uart is None:
            return False
            
        try:
            # バッファをクリア
            self.uart.flushInput()
            
            # BNO055 UARTプロトコル（0xAA, 0x00, register, length, value）
            cmd = bytes([0xAA, 0x00, register, 0x01, value])
            self.uart.write(cmd)
            time.sleep(0.01)
            
            # レスポンスを読み取り
            resp = self.uart.read(2)
            
            if len(resp) < 2:
                print(f"書き込みレスポンスが短すぎます: {len(resp)}バイト")
                return False
            
            if resp[0] != 0xEE:
                print(f"書き込み失敗: レスポンス 0x{resp[0]:02X} (期待値: 0xEE)")
                return False
            
            if resp[1] != 0x01:
                print(f"書き込みステータス異常: 0x{resp[1]:02X} (期待値: 0x01)")
                return False
            
            return True
            
        except Exception as e:
            print(f"レジスタ書き込みエラー (0x{register:02X}): {e}")
            return False
    
    def get_vector(self, vector_type):
        """ベクトルデータを取得"""
        buf = self.read_register(vector_type, 6)
        if buf is None or len(buf) < 6:
            return None
        
        xyz = struct.unpack('<hhh', buf)
        
        # スケーリングファクタを適用
        if vector_type == self.VECTOR_MAGNETOMETER:
            scaling_factor = 16.0
        elif vector_type == self.VECTOR_GYROSCOPE:
            scaling_factor = 900.0
        elif vector_type == self.VECTOR_EULER:
            scaling_factor = 16.0
        elif vector_type == self.VECTOR_GRAVITY:
            scaling_factor = 100.0
        else:
            scaling_factor = 1.0
        
        return tuple([i / scaling_factor for i in xyz])
    
    def get_quaternion(self):
        """クォータニオンを取得"""
        buf = self.read_register(self.BNO055_QUATERNION_DATA_W_LSB_ADDR, 8)
        if buf is None or len(buf) < 8:
            return None
        
        wxyz = struct.unpack('<hhhh', buf)
        return tuple([i * (1.0 / (1 << 14)) for i in wxyz])
    
    def get_calibration(self):
        """キャリブレーション状態を取得"""
        cal_data = self.read_register(self.BNO055_CALIB_STAT_ADDR)
        if cal_data is None:
            return None
        
        sys = (cal_data >> 6) & 0x03
        gyro = (cal_data >> 4) & 0x03
        accel = (cal_data >> 2) & 0x03
        mag = cal_data & 0x03
        
        return (sys, gyro, accel, mag)
    
    def get_temperature(self):
        """温度を取得"""
        return self.read_register(self.BNO055_TEMP_ADDR)
    
    def get_system_status(self):
        """システム状態を取得"""
        sys_stat = self.read_register(self.BNO055_SYS_STAT_ADDR)
        self_test = self.read_register(self.BNO055_SELFTEST_RESULT_ADDR)
        sys_err = self.read_register(self.BNO055_SYS_ERR_ADDR)
        
        return (sys_stat, self_test, sys_err)
    
    def close(self):
        """シリアルポートを閉じる"""
        if self.uart:
            self.uart.close()
            self.uart = None


def main():
    """メイン関数"""
    print("BNO055テストプログラム開始")
    print("ポート: /dev/ttyUSB0")
    print("Ctrl+Cで終了します")
    
    # BNO055を初期化（まずIMUPLUSモードで試す）
    bno = BNO055('/dev/ttyUSB0')
    
    if not bno.begin(BNO055.OPERATION_MODE_IMUPLUS):
        print("BNO055の初期化に失敗しました")
        print("デバイス接続とボーレートを確認してください")
        return
    
    try:
        print("データ取得を開始します...")
        loop_count = 0
        
        while True:
            loop_count += 1
            print(f"\n--- Loop {loop_count} ---")
            
            # まず基本的なデータから取得
            # オイラー角を取得
            euler = bno.get_vector(BNO055.VECTOR_EULER)
            if euler:
                heading, roll, pitch = euler
                print(f"オイラー角: H={heading:6.1f}° R={roll:6.1f}° P={pitch:6.1f}°")
            else:
                print("オイラー角: 取得失敗")
            
            # 加速度を取得（重力の確認）
            accel = bno.get_vector(BNO055.VECTOR_ACCELEROMETER)
            if accel:
                x, y, z = accel
                print(f"加速度:     X={x:6.2f} Y={y:6.2f} Z={z:6.2f} m/s²")
            else:
                print("加速度: 取得失敗")
            
            # キャリブレーション状態を確認
            cal = bno.get_calibration()
            if cal:
                sys, gyro, accel, mag = cal
                print(f"キャリブレーション: Sys={sys} Gyro={gyro} Accel={accel} Mag={mag}")
            else:
                print("キャリブレーション: 取得失敗")
            
            # 温度
            temp = bno.get_temperature()
            if temp is not None:
                print(f"温度: {temp}°C")
            else:
                print("温度: 取得失敗")
            
            # 5回ごとに詳細情報を表示
            if loop_count % 5 == 0:
                print("\n詳細センサ情報:")
                
                # ジャイロを取得
                gyro = bno.get_vector(BNO055.VECTOR_GYROSCOPE)
                if gyro:
                    x, y, z = gyro
                    print(f"  ジャイロ:   X={x:6.2f} Y={y:6.2f} Z={z:6.2f} rad/s")
                
                # 磁気を取得
                mag = bno.get_vector(BNO055.VECTOR_MAGNETOMETER)
                if mag:
                    x, y, z = mag
                    print(f"  磁気:       X={x:6.1f} Y={y:6.1f} Z={z:6.1f} µT")
                
                # システム状態を取得
                status = bno.get_system_status()
                if status:
                    sys_stat, self_test, sys_err = status
                    print(f"  システム状態: Status=0x{sys_stat:02X} SelfTest=0x{self_test:02X} Error=0x{sys_err:02X}")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n\nプログラムを終了します")
    
    finally:
        bno.close()
        print("BNO055接続を閉じました")


if __name__ == "__main__":
    main()