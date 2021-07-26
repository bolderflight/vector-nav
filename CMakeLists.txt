cmake_minimum_required(VERSION 3.13)
if (DEFINED MCU)
	# Setting up the toolchain
	set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
endif()
# Project information
project(VectorNav
	VERSION 2.0.1
	DESCRIPTION "VectorNav driver"
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
		GIT_REPOSITORY 	https://github.com/bolderflight/core.git
		GIT_TAG v2.0.5
	)
	FetchContent_MakeAvailable(core)
	# Fetch eigen
	FetchContent_Declare(
		eigen
		GIT_REPOSITORY 	https://github.com/bolderflight/eigen.git
		GIT_TAG v2.0.0
	)
	FetchContent_MakeAvailable(eigen)
	# Fetch units
	FetchContent_Declare(
		units
		GIT_REPOSITORY 	https://github.com/bolderflight/units.git
		GIT_TAG v3.2.0
	)
	FetchContent_MakeAvailable(units)
	# Add the library target
	add_library(vector_nav
		include/vector_nav/vector_nav.h
		include/vector_nav/registers.h
		include/vector_nav/vn.h
		include/vector_nav/vn100.h
		include/vector_nav/vn200.h
		include/vector_nav/vn300.h
		src/vector_nav/vn100.cc
		src/vector_nav/vn200.cc
		src/vector_nav/vn300.cc
	)
	# Link libraries
	target_link_libraries(vector_nav
		PUBLIC
			core
			eigen
			units
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
