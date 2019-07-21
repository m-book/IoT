import time
import uuid
import Adafruit_BluefruitLE
import MySQLdb

SERVICE_UUID = uuid.UUID('4fafc201-1fb5-459e-8fcc-c5c9c331914b')
CHAR_UUID    = uuid.UUID('beb5483e-36e1-4688-b7f5-ea07361b26a8')

ble = Adafruit_BluefruitLE.get_provider()


class Temperature:
    MAX_TEMPERATURE_COUNT = 77
    THRESHOLD_COUNT = 7
    def __init__(self):
        self.temperature_list = []
        self.is_exist = False
        self.mysql = MySQL()
        # self.mysql.create_table()

    def receive_data(self, data):
        self.temperature_list.append(float(data))
        if len(self.temperature_list) >= self.MAX_TEMPERATURE_COUNT:
            self.check()
            self.temperature_list = []

    def check(self):
        print("check")
        n = len([x for x in self.temperature_list if 30 <= x <= 40])
        if n < self.THRESHOLD_COUNT and self.is_exist:
            self.mysql.add("left")
            self.is_exist = False
        elif n >= self.THRESHOLD_COUNT and self.is_exist is False:
            self.mysql.add("append")
            self.is_exist = True



class MySQL:
    def __init__(self):
        # データベースへの接続とカーソルの生成
        self.connection = MySQLdb.connect(
            host='localhost',
            user='test',
            passwd='testtest',
            db='iot_test')
        self.cursor = self.connection.cursor()


    def create_table(self):
        self.cursor.execute(
            """CREATE TABLE iot (
                id INT AUTO_INCREMENT NOT NULL PRIMARY KEY,
                status VARCHAR(50),
                created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
            );"""
        )
        # 保存を実行
        self.connection.commit()

    def add(self, status):
        # データの追加
        print(status)
        self.cursor.execute("""INSERT INTO iot (status)
            VALUES ('{}');
            """.format(status))
        # 保存を実行
        self.connection.commit()

    def close(self):
        # 接続を閉じる
        self.connection.close()

def main():
    temperature = Temperature()

    ble.clear_cached_data()

    adapter = ble.get_default_adapter()
    adapter.power_on()
    print('Using adapter: {0}'.format(adapter.name))

    print('Disconnecting any connected devices...')
    ble.disconnect_devices([SERVICE_UUID])

    # Scan devices.
    print('Searching device...')
    try:
        adapter.start_scan()

        device = ble.find_device(name="m5-stack")
        if device is None:
            raise RuntimeError('Failed to find device!')
    finally:
        # Make sure scanning is stopped before exiting.
        adapter.stop_scan()

    print('Connecting to device...')
    device.connect()  # Will time out after 60 seconds, specify timeout_sec parameter
                      # to change the timeout.

    # Once connected do everything else in a try/finally to make sure the device
    # is disconnected when done.
    try:
        print('Discovering services...')
        device.discover([SERVICE_UUID], [CHAR_UUID])

        # Find the service and its characteristics.
        uart = device.find_service(SERVICE_UUID)
        chara = uart.find_characteristic(CHAR_UUID)

        print('Message from device...')
        v = chara.read_value()
        print(v)

        # Write a string to the characteristic.
        print('Sending message to device...')
        chara.write_value(b'hi!')

        def received(data):
            # print('Received: {0}'.format(data))
            temperature.receive_data(data)

        # Turn on notification of characteristics using the callback above.
        print('Subscribing to characteristic changes...')
        chara.start_notify(received)

        # Now just wait for 30 seconds to receive data.
        print('Waiting 600 seconds to receive data from the device...')
        time.sleep(600)
    finally:
        # Make sure device is disconnected on exit.
        device.disconnect()


# Initialize the BLE system.  MUST be called before other BLE calls!
ble.initialize()

# Start the mainloop to process BLE events, and run the provided function in
# a background thread.  When the provided main function stops running, returns
# an integer status code, or throws an error the program will exit.
ble.run_mainloop_with(main)





