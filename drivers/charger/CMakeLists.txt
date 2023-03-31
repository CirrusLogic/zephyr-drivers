# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SBS_CHARGER sbs_charger)

zephyr_library_sources_ifdef(CONFIG_USERSPACE charger_syscall_handlers.c)
