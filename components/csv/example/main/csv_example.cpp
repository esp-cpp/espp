#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include "csv.hpp" // includes csv2/reader.hpp and csv2/writer.hpp
#include "format.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    fmt::print("Starting csv reader example!\n");
    //! [csv reader example]
    std::string csv_data =
        "filename, boxart filename, display name\n"
        "mario.nes, boxart/mario.jpg, Mario Bros.\n"
        "super_mario_1.nes, boxart/super_mario_bros_1.jpg, Super Mario Bros.\n"
        "super_mario_3.nes, boxart/super_mario_bros_3.jpg, Super Mario Bros. 3\n"
        "zelda.nes, boxart/zelda1.jpg, The Legend of Zelda\n"
        "zelda_2.nes, boxart/zelda2.jpg, The Legend of Zelda 2: the Adventure of Link\n"
        "mega_man.nes, boxart/megaman1.jpg, MegaMan\n"
        "metroid.nes, boxart/metroid1.jpg, Metroid\n"
        "pokemon_blue.gb, boxart/pokemon_blue.jpg, Pokemon Blue\n"
        "pokemon_red.gb, boxart/pokemon_red.jpg, Pokemon Red\n"
        "pokemon_yellow.gbc, boxart/pokemon_yellow.jpg, Pokemon Yellow\n"
        "links_awakening.gb, boxart/tloz_links_awakening.jpg, The Legend of Zelda: Link's "
        "Awakening\n"
        "links_awakening.gbc, boxart/tloz_links_awakening_dx.jpg, The Legend of Zelda: Link's "
        "Awakening DX";

    // create the csv reader object
    csv2::Reader<csv2::delimiter<','>, csv2::quote_character<'"'>, csv2::first_row_is_header<true>,
                 csv2::trim_policy::trim_whitespace>
        csv;
    if (csv.parse(csv_data)) {
      // print the header
      const auto header = csv.header();
      fmt::print("Header:\n\t");
      for (const auto cell : header) {
        std::string value;
        cell.read_value(value);
        fmt::print("'{}', ", value);
      }
      fmt::print("\n");
      // now print the items
      size_t row_index = 0;
      for (const auto row : csv) {
        fmt::print("Row {} values:\n\t", row_index);
        for (const auto cell : row) {
          std::string value;
          cell.read_value(value);
          fmt::print("'{}', ", value);
        }
        fmt::print("\n");
        row_index++;
      }
    }
    //! [csv reader example]
  }

  {
    fmt::print("Starting csv writer example!\n");
    //! [csv writer example]
    std::ostringstream stream("");
    csv2::Writer<csv2::delimiter<','>, std::ostringstream> writer(stream);

    std::vector<std::vector<std::string>> rows = {
        {"a", "b", "c"}, {"1", "2", "3"}, {"4", "5", "6"}};

    writer.write_rows(rows);
    fmt::print("Wrote:\n'{}'\n", stream.str());
    //! [csv writer example]
  }

  fmt::print("CSV example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
