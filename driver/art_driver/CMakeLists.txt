# Created by Steven Zhang on 19-02-14.
# art racecar

cmake_minimum_required(VERSION 2.8.3)
project(art_driver)


include_directories(
   include
  
)





 add_executable(${PROJECT_NAME} src/art_racecar.cpp
         include/art_racecar_driver.h
         )






 target_link_libraries(${PROJECT_NAME}
      ${PROJECT_SOURCE_DIR}/lib/libart_driver.a
   
 )


