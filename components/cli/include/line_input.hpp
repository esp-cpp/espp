#pragma once

#if defined(ESP_PLATFORM)

#include <sdkconfig.h>

#if CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)

#include <algorithm>
#include <atomic>
#include <deque>
#include <functional>
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
 * this does not run continuously, but only when the user presses a key. It will
 * only detect a resize when the user presses a key, so if the user resizes the
 * terminal without pressing a key, it will not be detected. This is a bit of a
 * hack, but it seems to work ok enough for now. NOTE: this feature is enabled
 * by default, but can be disabled by calling set_handle_resize(false).
 *
 * @warning The handle resize functionality is not very robust and can sometimes
 *          result in the prompt thinking that it was resized when it was not.
 *          Use with caution. This seems to happen if you hold the enter key
 *          down for too long. If this happens, you can press ctrl+l to redraw
 *          the prompt and input.
 */
class LineInput {
public:
  /// function for printing the prompt if there is one
  typedef std::function<void(void)> prompt_fn;

  /// function for getting completions for a given input
  typedef std::function<std::vector<std::string>(const std::string &)> get_completions_fn;

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
  const History &get_history() const { return input_history_; }

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
   * @brief Set whether or not to handle terminal resize events.
   * @note If \p handle_resize is true, then the terminal will be cleared and
   *       the prompt and input will be redrawn any time the terminal is
   *       resized.
   * @warning This is not very robust and should be improved. Use with caution.
   * @param handle_resize Whether or not to handle terminal resize events.
   */
  void set_handle_resize(bool handle_resize) { should_handle_resize_ = handle_resize; }

  /**
   * @brief Set whether or not to send escape sequences.
   * @note If \p send_escape_sequences is true, then escape sequences will be
   *       sent to the terminal. If false, they will not be sent. Escape
   *       sequences are used for things like moving the cursor around the
   *       terminal, clearing the screen, and getting the cursor position.
   * @param send_escape_sequences Whether or not to send escape sequences.
   */
  void set_send_escape_sequences(bool send_escape_sequences) {
    send_escape_sequences_ = send_escape_sequences;
  }

  /**
   * @brief Set whether or not to handle control commands.
   * @note If \p handle_control_commands is true, then control commands will be
   *       handled. If false, they will not be handled. Control commands are
   *       things like backspace, arrow keys, and CTRL+<KEY> input commands.
   *       If this is false, then all data received will be treated as input
   *       characters.
   * @param handle_control_commands Whether or not to handle control commands.
   */
  void set_handle_control_commands(bool handle_control_commands) {
    handle_control_commands_ = handle_control_commands;
  }

  /**
   * @brief Get the current terminal size.
   * @note Tries to move the cursor to the bottom right of the terminal
   *       (999,999) and then get the cursor position. This is a bit of a hack,
   *       but it seems to work.
   * @note If send_escape_sequences is false, then the terminal width and height
   *       will be returned as the last known values, which may not be accurate.
   * @param width Reference to an int to store the width in.
   * @param height Reference to an int to store the height in.
   */
  void get_terminal_size(int &width, int &height) const {
    if (!send_escape_sequences_) {
      width = terminal_width_;
      height = terminal_height_;
      return;
    }
    printf("\033[s\033[999;999H\033[6n\033[u");
    fflush(stdout);
    fsync(fileno(stdout));
    scanf("\033[%d;%dR", &height, &width);
  }

  /**
   * @brief Get user input with arrow key and backspace support
   * @param is Reference to a std::istream from which to read input
   * @param prompt Function to show prompt at the beginning of the line
   *        (optional)
   * @param get_completions Function to get completions for the current input
   *        (optional)
   * @return User input as a std::string
   */
  std::string get_user_input(std::istream &is, prompt_fn prompt = nullptr,
                             get_completions_fn get_completions = nullptr) {
    int start_pos_x = 0, start_pos_y = 0;
    get_cursor_position(start_pos_x, start_pos_y);

    if (should_handle_resize_) {
      // get the current terminal size
      get_terminal_size(terminal_width_, terminal_height_);
    }

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

      // if we don't handle control commands, then just treat everything as
      // input characters
      if (!handle_control_commands_) {
        if (ch == '\n') { // Enter
          fmt::print("\n");
          break;
        } else { // Regular character
          input.insert(input.begin() + pos_x - start_pos_x, ch);
          std::cout << input.substr(pos_x - start_pos_x);
          pos_x++;
        }
        continue;
      }

      // if we are here, then we are handling control commands

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
        default:
          // we likely got some other escape sequence, so just ignore it
          {
            // ignore the rest of the sequence; it likely came from our calls to
            // get_cursor_position and get_terminal_size which expect a response
            // of the form \033[#;#R so we'll ignore until we see the ';' and
            // then 'R'
            is.ignore(std::numeric_limits<std::streamsize>::max(), ';');
            is.ignore(std::numeric_limits<std::streamsize>::max(), 'R');
          }
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
      } else if (ch == '\t') {
        // handle tab key for completions, printing them all in one line if there are multiple
        // or just inserting the completion if there is only one
        auto current_line = input;
        auto completions =
            get_completions ? get_completions(current_line) : std::vector<std::string>();
        if (completions.size() == 1) {
          // need to clear the line and redraw the prompt and input, and move
          // the cursor to the end
          clear_line();
          input = completions[0];
          redraw(start_pos_x, input, prompt);
          pos_x = start_pos_x + input.size();
        } else if (completions.size() > 1) {
          // print all the completions in one line below the current input,
          // then move the cursor back to the input line
          std::cout << std::endl;
          // make sure to clear to the end of the line
          clear_to_end_of_line();
          for (const auto &completion : completions) {
            std::cout << completion << " ";
          }
          // need to handle the case where we were at the bottom of the screen,
          // so we need to move the cursor up one line before printing the
          // completions and then move it back down after
          if (pos_y == terminal_height_) {
            pos_y--;
          }
        }
      } else if (ch == '\n') { // Enter
        // print a new line to move to the next line, since this was the end
        // of input
        fmt::print("\n");
        // and clear the line from the cursor to the end
        clear_to_end_of_line();
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
  void clear_screen() const {
    if (!send_escape_sequences_)
      return;
    printf("\033[2J"); // Clear the screen
  }

  /**
   * @brief Clear the line (that the cursor is on)
   */
  void clear_line() const {
    if (!send_escape_sequences_)
      return;
    printf("\033[2K"); // Clear (0) cursor to end of line, (1), cursor to start of line, or (2)
                       // entire line
  }

  /**
   * @brief Clear to end of line (from cursor)
   */
  void clear_to_end_of_line() const {
    if (!send_escape_sequences_)
      return;
    printf("\033[0K"); // Clear (0) cursor to end of line, (1), cursor to start of line, or (2)
                       // entire line
  }

  /**
   * @brief Clear to start of line (from cursor)
   */
  void clear_to_start_of_line() const {
    if (!send_escape_sequences_)
      return;
    printf("\033[1K"); // Clear (0) cursor to end of line, (1), cursor to start of line, or (2)
                       // entire line
  }

protected:
  void redraw(int start_pos_x, std::string_view input, prompt_fn prompt) const {
    if (!send_escape_sequences_)
      return;

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
  void move_cursor(int x, int y) const {
    if (!send_escape_sequences_)
      return;
    printf("\033[%d;%dH", y, x);
  }

  void move_cursor(int x) const {
    if (!send_escape_sequences_)
      return;
    printf("\033[%dG", x);
  }

  // Get cursor position
  void get_cursor_position(int &x, int &y) const {
    if (!send_escape_sequences_)
      return;
    printf("\033[6n"); // Request cursor position
    fflush(stdout);
    fsync(fileno(stdout));
    scanf("\033[%d;%dR", &y, &x);
  }

  // Update the terminal size and return true if it changed
  bool handle_resize() {
    if (!send_escape_sequences_)
      return false;
    if (!should_handle_resize_)
      return false;
    int term_width, term_height;
    get_terminal_size(term_width, term_height);
    if (term_width != terminal_width_ || term_height != terminal_height_) {
      terminal_width_ = term_width;
      terminal_height_ = term_height;
      return true;
    }
    return false;
  }

  int terminal_width_{80};
  int terminal_height_{24};
  size_t history_size_ = 0;
  History input_history_;
  std::atomic<bool> should_handle_resize_{true};
  std::atomic<bool> send_escape_sequences_{true};
  std::atomic<bool> handle_control_commands_{true};
};
} // namespace espp

#endif // CONFIG_COMPILER_CXX_EXCEPTIONS

#endif // ESP_PLATFORM
