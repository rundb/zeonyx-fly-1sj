cmake_minimum_required(VERSION 3.20.0)

# Expose local boards/ directory as an out-of-tree board root
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(1sj_shell)

target_sources(app PRIVATE
	src/main.c
	src/task_sx1262.c
)
