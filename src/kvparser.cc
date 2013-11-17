/*
 * kvparser.cc
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt.
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * Implementation. See kvparser.hh.
 */

#include <vector>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "kvparser.hh"

using namespace std;

namespace textured_localization
{

KVParser::KVParser()
  : _map()
{
}

KVParser::KVParser(const string& filename)
  : _map()
{
  int linen = 1;
  ifstream file(filename.c_str());
  if (!file)
    throw (boost::format("KVParser couldn't open %s") % filename).str();

  string line;
  while (getline(file, line))
  {
    // No '=' => skip it.
    if (line.find('=') == line.npos)
    {
      linen++;
      continue;
    }

    // Drop everything after a comment character.
    vector<string> hash_split;
    boost::split(hash_split, line, boost::is_any_of("#"));

    // If the first part is empty, skip that too.
    boost::trim(hash_split[0]);
    if (hash_split[0] == "")
    {
      linen++;
      continue;
    }

    // Split key = value into what we want.
    vector<string> equal_split;
    boost::split(equal_split, hash_split[0], boost::is_any_of("="));
    if (equal_split.size() != 2)
    {
      cout << "SIZE " << equal_split.size() << endl;
      for (size_t i = 0 ; i < equal_split.size() ; ++i)
        cout << i << ": " << equal_split[i] << endl;
      cout << flush;
      throw (boost::format("KV parsing failure on line %d!") % linen).str();
    }
    
    // Whitespace is bad!
    boost::trim(equal_split[0]);
    boost::trim(equal_split[1]);

    _map.insert(make_pair(equal_split[0], equal_split[1]));
    linen++;
  }
}

KVParser::KVParser(const KVParser& rhs)
  : _map(rhs._map)
{
}

KVParser& KVParser::operator=(const KVParser& rhs)
{
  if (this == &rhs)
    return *this;

  _map = rhs._map;
  return *this;
}

KVParser::~KVParser()
{
}

string KVParser::operator[](const string& idx) 
{
  map<string, string>::iterator i = _map.find(idx);
  if (i == _map.end())
    return "";
  else
    return i->second;
}



}
