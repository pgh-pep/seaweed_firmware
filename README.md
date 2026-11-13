# seaweed_firmware


to add common to path, update cmakelists.txt:
```cmake
# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
)
```


can: 500 kbps, 87.50% sample point
 https://embeddedcalc.com/tools/can-bit-timing-calculator/