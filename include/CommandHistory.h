#pragma once
#include <Arduino.h>
#include <array>

template <size_t SIZE = 16> class CommandHistory
{
public:
    CommandHistory() : _head(0), _count(0), _cursor(-1) {}

    // add new command
    void push(const String& cmd)
    {
        if (_count > 0 && _buffer[(_head - 1 + SIZE) % SIZE] == cmd)
            return;  // don't save repeated commands

        _buffer[_head] = cmd;
        _head          = (_head + 1) % SIZE;
        if (_count < SIZE)
            _count++;
        _cursor = -1;  // reset cursor after push
    }

    // go back (Arrow Up)
    String up()
    {
        if (_count == 0)
            return "";

        if (_cursor < (int)_count - 1)
            _cursor++;
        return _buffer[(_head - 1 - _cursor + SIZE) % SIZE];
    }

    // go forward (Arrow Down)
    String down()
    {
        if (_cursor > 0)
        {
            _cursor--;
            return _buffer[(_head - 1 - _cursor + SIZE) % SIZE];
        }
        else
        {
            _cursor = -1;
            return "";  // return to empty line
        }
    }

    void resetCursor()
    {
        _cursor = -1;
    }

private:
    std::array<String, SIZE> _buffer;
    size_t                   _head;    // next write location
    size_t                   _count;   // actual item count
    int                      _cursor;  // for navigation: -1 = no selection
};
