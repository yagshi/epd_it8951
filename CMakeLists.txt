cmake_minimum_required(VERSION 3.1)
project(libit8951 C)
add_library(it8951 SHARED it8951.c it8951.h it8951_p.h)
add_executable(main test.c it8951.h)
target_link_libraries(it8951 bcm2835)
target_link_libraries(main it8951)
