board_runner_args(jlink "--device=MSPM0G3507" "--iface=SWD" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
