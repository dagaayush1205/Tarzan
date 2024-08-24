if(CONFIG_R2-D2)
    set(R2-D2_DIR ${ZEPHYR_CURRENT_MODULE_DIR})
    zephyr_include_directories(${R2-D2_DIR}/include)
    zephyr_library()
    zephyr_library_sources(
        ${R2-D2_DIR}/lib/drive.c
	${R2-D2_DIR}/lib/sbus.c
   
        )
endif()
