/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file assert.h
 * @author W. Nicholas Greene
 * @date 2017-02-08 12:27:19 (Wed)
 */

#pragma once

#ifdef DATASET_UTILS_NO_ASSERT
#define DATASET_UTILS_ASSERT(x)
#else
#include <cxxabi.h>
#include <unistd.h>
#include <execinfo.h> // for stack trace
#include <cstdlib>   // for abort

namespace dataset_utils {

inline void assert_fail(const char *condition, const char *function,
                        const char *file, int line) {
  fprintf(stderr, "DATASET_UTILS_ASSERT failed: %s in function %s at %s: %i\n",
          condition, function, file, line);

  fprintf(stderr, "Stacktrace:\n");

  // Get and print stack trace with demangled names. Taken from:
  // https://panthema.net/2008/0901-stacktrace-demangled
  constexpr int max_frames = 16;
  void* stack_frames[max_frames];
  int num_frames = backtrace(stack_frames, max_frames); // Get stack addresses.

  // Get strings of trace.
  char** symbols = backtrace_symbols(stack_frames, num_frames);

  // Allocate string which will be filled with the demangled function name.
  // allocate string which will be filled with the demangled function name
  size_t funcnamesize = 256;
  char* funcname = static_cast<char*>(malloc(funcnamesize));

  // iterate over the returned symbol lines. skip the first, it is the
  // address of this function.
  for (int i = 1; i < num_frames; i++) {
    char *begin_name = 0, *begin_offset = 0, *end_offset = 0;

    // find parentheses and +address offset surrounding the mangled name:
    // ./module(function+0x15c) [0x8048a6d]
    for (char* p = symbols[i]; *p; ++p) {
      if (*p == '(') {
        begin_name = p;
      } else if (*p == '+') {
        begin_offset = p;
      } else if (*p == ')' && begin_offset) {
        end_offset = p;
        break;
      }
    }

    if (begin_name && begin_offset && end_offset && begin_name < begin_offset) {
      *begin_name++ = '\0';
      *begin_offset++ = '\0';
      *end_offset = '\0';

      // mangled name is now in [begin_name, begin_offset) and caller
      // offset in [begin_offset, end_offset). now apply
      // __cxa_demangle():

      int status;
      char* ret = abi::__cxa_demangle(begin_name, funcname, &funcnamesize,
                                      &status);
      if (status == 0) {
        funcname = ret; // use possibly realloc()-ed string
        fprintf(stderr, "  %s : %s+%s\n", symbols[i], funcname, begin_offset);
      } else {
        // demangling failed. Output function name as a C function with
        // no arguments.
        fprintf(stderr, "  %s : %s()+%s\n", symbols[i], begin_name, begin_offset);
      }
    } else {
      // couldn't parse the line? print the whole line.
      fprintf(stderr, "  %s\n", symbols[i]);
    }
  }

  free(funcname);
  free(symbols);

  abort();
}

}  // namespace dataset_utils

#define DATASET_UTILS_ASSERT(condition) \
  do { \
    if (!(condition)) \
      dataset_utils::assert_fail(#condition, __PRETTY_FUNCTION__, __FILE__, __LINE__); \
  } while (false)

#endif
