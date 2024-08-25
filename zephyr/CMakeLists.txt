if(CONFIG_Tarzan)
    set(Tarzan_DIR ${ZEPHYR_CURRENT_MODULE_DIR})
    zephyr_include_directories(${Tarzan_DIR}/include)
    zephyr_library()
    zephyr_library_sources(
        ${Tarzan_DIR}/lib/drive.c
	${Tarzan_DIR}/lib/sbus.c
   
        )
endif()
