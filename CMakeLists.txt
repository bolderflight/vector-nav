cmake_minimum_required(VERSION 3.13)
if (DEFINED MCU)
	# Setting up the toolchain
	set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
endif()
# Project information
project(VectorNav
	VERSION 1.0.0
	DESCRIPTION "VectorNav INS driver"
	LANGUAGES CXX
)
if (DEFINED MCU)
	# Grab the processor and set up definitions and compile options
	include(${CMAKE_SOURCE_DIR}/cmake/config_mcu.cmake)
	configMcu(${MCU})
	include(FetchContent)
	# Fetch core
	FetchContent_Declare(
		core
		GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/core.git
	)
	FetchContent_MakeAvailable(core)
	# Fetch eigen
	FetchContent_Declare(
		eigen
		GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/eigen.git
	)
	FetchContent_MakeAvailable(eigen)
	# Fetch global defs
	FetchContent_Declare(
		global_defs
		GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/global_defs.git
	)
	FetchContent_MakeAvailable(global_defs)
	# Add the library target
	add_library(vector_nav
		src/vector_nav/vector_nav.cc
		include/vector_nav/vector_nav.h
	)
	# Link libraries
	target_link_libraries(vector_nav
		PUBLIC
			core
			eigen
			global_defs
	)
	# Setup include directories 
	target_include_directories(vector_nav PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	)
endif()

# Example and test if this project is built separately
if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
	if (DEFINED MCU)		
		# Add the spi example target
		add_executable(spi_example examples/spi.cc)
		# Add the includes
		target_include_directories(spi_example PUBLIC 
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<INSTALL_INTERFACE:include>
		)
		# Link libraries to the example target
		target_link_libraries(spi_example
			PRIVATE 
			vector_nav
		)
		# Add hex and upload targets
		include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
		FlashMcu(spi_example ${MCU})
	endif()
endif()
