#
# Copyright (c) 2019, Christian Taedcke
#
# SPDX-License-Identifier: Apache-2.0
#

board_runner_args(jlink "--device=SiM3U167")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
