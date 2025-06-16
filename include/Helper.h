#ifndef HELPER_H
#define HELPER_H

#include <Arduino.h>

class Helper
{
public:
    inline static String greenText(const String& text)
    {
        return "\033[32m" + text + "\033[0m";
    }

    // Alternative method using const char* for better memory efficiency
    inline static String greenText(const char* text)
    {
        return String("\033[32m") + text + "\033[0m";
    }

    //////////////////////////////////////////////////////////////////////////////
    inline static String redText(const String& text)
    {
        return "\033[31m" + text + "\033[0m";
    }

    // Alternative method using const char* for better memory efficiency
    inline static String redText(const char* text)
    {
        return String("\033[31m") + text + "\033[0m";
    }
};

#endif  // HELPER_H