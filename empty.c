/*-
 * $Copyright$
 */

/*
 * This file is intentionally left blank:
 * 
 * CMake will complain if defining targets (executables, libraries) without
 * associated source files.
 * 
 * However, for the Unit Test Build, a bunch of libraries need to be stubbed
 * out. This is done by "building" a library with the same name, but without
 * content. This allows for the CMakeLists.txt files that attempt to use the
 * stubbed-out libraries to remain unchanged.
 */
