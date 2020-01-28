cmake_minimum_required(VERSION 2.6)

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
find_package(Intl REQUIRED)
if (Intl_FOUND)
   include_directories (${Intl_INCLUDE_DIRS})
endif()

if (CURSES_HAVE_CURSES_H)
   add_definitions(-DHAVE_CURSES_H=1)
endif ()
if (CURSES_HAVE_NCURSES_H)
   add_definitions(-DHAVE_NCURSES_H=1)
endif ()
if (CURSES_HAVE_NCURSES_CURSES_H)
   add_definitions(-DHAVE_NCURSES_CURSES_H=1)
endif ()
if (CURSES_HAVE_NCURSES_NCURSES_H)
   add_definitions(-DHAVE_NCURSES_NCURSES_H=1)
endif ()

include_directories(
   "${PROJECT_SOURCE_DIR}/lxdialog"
   ${CURSES_INCLUDE_DIR}
   ${Intl_INCLUDE_DIRS}
)
add_subdirectory(lxdialog)

add_library(zconf zconf.tab.c)

add_executable(mconf mconf.c)
add_executable(conf conf.c)

target_link_libraries(mconf lxdialog ${CURSES_LIBRARIES} ${Intl_LIBRARIES} zconf)
target_link_libraries(conf ${CURSES_LIBRARIES} ${Intl_LIBRARIES} zconf)
