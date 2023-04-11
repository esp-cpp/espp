#include <algorithm>
#include <deque>
#include <iostream>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include "format.hpp"

namespace espp {
/**
 * @brief Class for getting a line of input from the terminal using c++
 *        istream while showing the input and allowing cursor navigation and
 *        backspace. Optionally allows for a prompt to be printed and command
 *        history to be stored.
 *
 *        The class allows for line movement using:
 *        *   ctrl+a (move to beginning of line)
 *        *   ctrl+e (move to end of line)
 *        *   ctrl+n (move up a line / previous input history)
 *        *   ctrl+p (move down a line / next input history)
 *        *   ctrl+k (delete from the cursor to the end of the line)
 */
class LineInput {
public:
  /// function for printing the prompt if there is one
  typedef std::function<void(void)> prompt_fn;

  /// Constructor
  LineInput() {}

  /// Destructor
  ~LineInput() {}

  /**
   * @brief Set the history size for the line input.
   * @note If the current history is larger, it will be resized, losing the
   *       oldest history.
   * @param new_size The new number of lines of history to store in memory.
   */
  void set_history_size(size_t new_size) {
    history_size_ = std::max(new_size, size_t(1));
    if (input_history_.size() > history_size_)
      input_history_.resize(history_size_);
  }

  /**
   * @brief Get user input with arrow key and backspace support
   * @param is Reference to a std::istream from which to read input
   * @param prompt Function to show prompt at the beginning of the line
   * @return User input as a std::string
   */
  std::string get_user_input(std::istream &is, prompt_fn prompt = nullptr) {
    int start_pos_x, start_pos_y;
    get_cursor_position(start_pos_x, start_pos_y);

    // add a new element to the front of the queue
    std::string &input = input_history_.emplace_front();
    // and remove the oldest input if we're over the allowed size
    if (input_history_.size() > history_size_) {
      input_history_.pop_back();
    }

    int pos_x = start_pos_x, pos_y = start_pos_y;
    int input_index = 0;

    while (true) {
      int ch = is.get();

      // Handle arrow keys
      if (ch == '\033') {
        is.get(); // Skip '['
        switch (is.get()) {
        case 'A': // Up
          pos_y = std::max(start_pos_y, pos_y - 1);
          input_index = std::min(input_index + 1, int(input_history_.size() - 1));
          input = input_history_[input_index];
          redraw(start_pos_x, input, prompt);
          break;
        case 'B': // Down
          pos_y++;
          input_index = std::max(input_index - 1, 0);
          input = input_history_[input_index];
          redraw(start_pos_x, input, prompt);
          break;
        case 'C': // Right
          pos_x = std::min((int)input.size() + start_pos_x, pos_x + 1);
          break;
        case 'D': // Left
          pos_x = std::max(start_pos_x, pos_x - 1);
          break;
        }
      } else if (ch == 1) { // Ctrl+A (move to start of line)
        pos_x = start_pos_x;
      } else if (ch == 5) { // Ctrl+E (move to end of line)
        pos_x = (int)input.size() + start_pos_x;
      } else if (ch == 2) { // Ctrl+B (move backward 1 character)
        pos_x = std::max(start_pos_x, pos_x - 1);
      } else if (ch == 6) { // Ctrl+F (move forward 1 character)
        pos_x = std::min((int)input.size() + start_pos_x, pos_x + 1);
      } else if (ch == 11) { // Ctrl+K (kill to end of line)
        input.resize(pos_x - start_pos_x);
        printf("\033[0K"); // Clear (0) cursor to end of line, (1), cursor to start of line, or (2)
                           // entire line
      } else if (ch == 14) { // Ctrl+N (move down 1 line)
        input_index = std::max(input_index - 1, 0);
        input = input_history_[input_index];
        redraw(start_pos_x, input, prompt);
      } else if (ch == 16) { // Ctrl+P (move up 1 line)
        input_index = std::min(input_index + 1, int(input_history_.size() - 1));
        input = input_history_[input_index];
        redraw(start_pos_x, input, prompt);
      } else if (ch == 127 || ch == 8) { // Backspace
        if (!input.empty() && pos_x > start_pos_x) {
          input.erase(input.begin() + pos_x - start_pos_x - 1);
          redraw(start_pos_x, input, prompt);
          pos_x--;
        }
      } else if (ch == '\n') { // Enter
        // print a new line to move to the next line, since this was the end
        // of input
        fmt::print("\n");
        break;
      } else { // Regular character
        input.insert(input.begin() + pos_x - start_pos_x, ch);
        std::cout << input.substr(pos_x - start_pos_x);
        pos_x++;
      }

      move_cursor(pos_x, pos_y);
    }

    return input;
  }

protected:
  void redraw(int start_pos_x, std::string_view input, prompt_fn prompt) {
    printf("\033[2K");     // Clear (0) cursor to end of line, (1), cursor to start of line, or (2)
                           // entire line
    printf("\033[%dG", 0); // Move cursor to beginning of the line
    // make sure to regenerate the prompt if there was one
    if (prompt)
      prompt();
    printf("\033[%dG", start_pos_x); // Move cursor to beginning of the input
    std::cout << input;
  }

  // Move the cursor
  void move_cursor(int x, int y) { printf("\033[%d;%dH", y, x); }

  // Get cursor position
  void get_cursor_position(int &x, int &y) {
    printf("\033[6n"); // Request cursor position
    scanf("\033[%d;%dR", &y, &x);
  }

  size_t history_size_ = 1;
  std::deque<std::string> input_history_;
};
} // namespace espp
