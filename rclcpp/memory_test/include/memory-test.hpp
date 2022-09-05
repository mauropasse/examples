#pragma once

#include <string>
#include <iostream>
#include <iomanip>

#include <sys/resource.h>
#include <sys/types.h>

// Globals
struct rusage usage;
long last_rss = 0;

void print_header()
{
  using namespace std;

  cout << setw(22) << "Entity" <<
          setw(4) << "N" <<
          setw(10) << "RSS" <<
          setw(10) << "DELTA" << endl;
}

void print_rss(std::string description, size_t number)
{
  using namespace std;

  getrusage(RUSAGE_SELF, &usage);

  long current_rss = usage.ru_maxrss;
  long delta_rss =current_rss - last_rss;

  cout << setw(22) << description <<
          setw(4) << number <<
          setw(10) << current_rss <<
          setw(10) << delta_rss << endl;

  last_rss = usage.ru_maxrss;
}