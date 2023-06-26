#include <algorithm>
#include <deque>
#include <iostream>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include "format.hpp"

namespace espp {
/**
 * @brief Class for getting a line of input from the terminal using c++ istream
 *        while showing the input and allowing cursor navigation and backspace.
 *        Optionally allows for a prompt to be printed and command history to be
 *        stored. By default the history_size is 0, which is unlimited history.
 *
 *        The class allows for line movement using:
 *        *   ctrl+l (clear the screen)
 *        *   ctrl+a (move to beginning of line)
 *        *   ctrl+e (move to end of line)
 *        *   ctrl+n (move up a line / previous input history)
 *        *   ctrl+p (move down a line / next input history)
 *        *   ctrl+k (delete from the cursor to the end of the line)
 *        *   ctrl+b (move back one character)
 *        *   ctrl+f (move forward one character)
 *
 * It has some _very_ basic support for handling terminal resize events, but
 * this is not very robust and should be improved. For now, any time it detects
 * a resize, it will clear the screen and redraw the prompt and input. Note that
 * this does not run continuously, but only when the user presses a key.
 */
class LineInput {
public:
  /// function for printing the prompt if there is one
  typedef std::function<void(void)> prompt_fn;

  /// Storage for the input history as a double-ended queue of strings
  typedef std::deque<std::string> History;

  /// Constructor
  LineInput() = default;

  /// Destructor
  ~LineInput() = default;

  /**
   * @brief Set the history size for the line input.
   * @note If \p new_size is 0, then there will be no limit on the size of
   *       the input history.
   * @note If the current history is larger, it will be resized, losing the
   *       oldest history.
   * @param new_size The new number of lines of history to store in memory.
   */
  void set_history_size(size_t new_size) {
    history_size_ = new_size;
    if (history_size_ > 0 && input_history_.size() > history_size_)
      input_history_.resize(history_size_);
  }

  /**
   * @brief Get the input history.
   * @return The input that has been entered so far, as History.
   */
  History get_history() const { return input_history_; }

  /**
   * @brief Replace any existing input history with \p history.
   * @note If \p history is longer than the current history_size, it will be
   *       truncated (oldest removed) to have size equal to history_size.
   * @param history New History to use.
   */
  void set_history(const History &history) {
    input_history_ = history;
    if (history_size_ > 0 && input_history_.size() > history_size_)
      input_history_.resize(history_size_);
  }

  /**
   * @brief Get the current terminal size.
   * @note Tries to move the cursor to the bottom right of the terminal
   *       (999,999) and then get the cursor position. This is a bit of a hack,
   *       but it seems to work.
   * @param width Reference to an int to store the width in.
   * @param height Reference to an int to store the height in.
   */
  void get_terminal_size(int &width, int &height) {
    printf("\033[s\033[999;999H\033[6n\033[u");
    fflush(stdout);
    scanf("\033[%d;%dR", &height, &width);
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

    // get the current terminal size
    get_terminal_size(terminal_width_, terminal_height_);

    // add a new element to the front of the queue
    std::string &input = input_history_.emplace_front();
    // and remove the oldest input if we're over the allowed size
    if (history_size_ > 0 && input_history_.size() > history_size_) {
      input_history_.pop_back();
    }

    int pos_x = start_pos_x, pos_y = start_pos_y;
    int input_index = 0;

    while (true) {
      if (handle_resize()) {
        // for now, just clear the screen and redraw the prompt and input
        // TODO: handle resizing more gracefully
        clear_screen();
        pos_y = 1;
        move_cursor(pos_x, pos_y);
        redraw(start_pos_x, input, prompt);
      }

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
        clear_to_end_of_line();
      } else if (ch == 12) { // Ctrl+L (clear screen)
        clear_screen();
        pos_y = 1;
        move_cursor(pos_x, pos_y);
        redraw(start_pos_x, input, prompt);
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

  /**
   * @brief Clear the screen
   */
  void clear_screen() {
    printf("\033[2J"); // Clear the screen
  }

  /**
   * @brief Clear the line (that the cursor is on)
   */
  void clear_line() {
    printf("\033[2K"); // Clear (0) cursor to end of line, (1), cursor to start of line, or (2)
                       // entire line
  }

  /**
   * @brief Clear to end of line (from cursor)
   */
  void clear_to_end_of_line() {
    printf("\033[0K"); // Clear (0) cursor to end of line, (1), cursor to start of line, or (2)
                       // entire line
  }

  /**
   * @brief Clear to start of line (from cursor)
   */
  void clear_to_start_of_line() {
    printf("\033[1K"); // Clear (0) cursor to end of line, (1), cursor to start of line, or (2)
                       // entire line
  }

protected:
  void redraw(int start_pos_x, std::string_view input, prompt_fn prompt) {
    printf("\033[2K");     // Clear (0) cursor to end of line, (1), cursor to start of line, or (2)
                           // entire line
    printf("\033[%dG", 0); // Move cursor to beginning of the line
    // make sure to regenerate the prompt if there was one
    if (prompt)
      prompt();
    // Move cursor to beginning of the input
    move_cursor(start_pos_x);
    std::cout << input;
  }

  // Move the cursor
  void move_cursor(int x, int y) { printf("\033[%d;%dH", y, x); }

  void move_cursor(int x) { printf("\033[%dG", x); }

  // Get cursor position
  void get_cursor_position(int &x, int &y) {
    printf("\033[6n"); // Request cursor position
    scanf("\033[%d;%dR", &y, &x);
  }

  // Update the terminal size and return true if it changed
  bool handle_resize() {
    int term_width, term_height;
    get_terminal_size(term_width, term_height);
    if (term_width != terminal_width_ || term_height != terminal_height_) {
      terminal_width_ = term_width;
      terminal_height_ = term_height;
      return true;
    }
    return false;
  }

  int terminal_width_;
  int terminal_height_;
  size_t history_size_ = 0;
  History input_history_;
};
} // namespace espp
