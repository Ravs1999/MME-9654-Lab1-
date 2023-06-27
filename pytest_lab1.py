# SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import pytest
from pytest_embedded import Dut


@pytest.mark.esp32s3
@pytest.mark.generic
def test_lab1(dut: Dut) -> None:
    dut.expect_exact('lab1: Create DC motor')
    dut.expect_exact('lab1: Init pcnt driver to decode rotary signal')
    dut.expect_exact('lab1: Create PID control block')
    dut.expect_exact('lab1: Create a timer to do PID calculation periodically')
    dut.expect_exact('lab1: Enable motor')
    dut.expect_exact('lab1: Forward motor')
    dut.expect_exact('lab1: Start motor speed loop')
