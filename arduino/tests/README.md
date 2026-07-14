# Tests

建议继续保持“一个测试 = 一个独立 Arduino 草图文件夹”：

```text
tests/
├── test_00_blink/test_00_blink.ino
├── test_01_serial_print/test_01_serial_print.ino
├── test_02_serial_command/test_02_serial_command.ino
├── test_03_target_parser_debug/test_03_target_parser_debug.ino
├── test_04_motor_driver_test/test_04_motor_driver_test.ino
└── test_05_distance_sensor_test/test_05_distance_sensor_test.ino
```

不要把这些测试 `.ino` 放进主草图目录，否则 Arduino 会把它们与主程序一起编译，造成多个 `setup()` / `loop()` 冲突。

详细测试步骤见 `docs/test_checklist.txt`。
