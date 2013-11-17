/*
 * kvparser.hh
 * Mac Mason <mac@cs.duke.edu>
 * Provided under the terms of the included LICENSE.txt
 * http://www.cs.duke.edu/~parr/textured-localization
 *
 * It's fairly handy to have (key, value) pairs available in a file. All this
 * does is parse them, as follows:
 *
 * Any line that doesn't contain an '=' is ignored.
 * Anything after a '#' is ignored.
 *
 * All this stores is a map from strings to strings; parsing things into
 * values is your problem (may I suggest inheritance?)
 */

#ifndef TEXTURED_LOCALIZATION_KVPARSER_HH_INCLUDED
#define TEXTURED_LOCALIZATION_KVPARSER_HH_INCLUDED 1

#include <iostream>
#include <string>
#include <map>

namespace textured_localization
{
  class KVParser
  {
    public:
      KVParser();  // Empty map.
      KVParser(const std::string& filename);  // parse a file.
      KVParser(const KVParser& rhs);
      KVParser& operator=(const KVParser& rhs);
      ~KVParser();

      /*
       * Returns "" if there's no mapping. Yes, that's not completely correct.
       * Deal with it.
       */
      std::string operator[](const std::string& idx);

      // Output functions; see the constructor for input.
      std::ostream& operator<<(std::ostream& out) const;
      void WriteToFile(const std::string& filename) const;

    private:
      // Store the data.
      std::map<std::string, std::string> _map;
  };
  
}

#endif
