#include "pid_protocol.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

const char *PID_ProtocolParse(const char *line, float values[PID_PARAMETER_COUNT])
{
    static const double limits[PID_PARAMETER_COUNT] =
        {1000.0, 10.0, 100.0, 1.0, 45.0, 1000.0, 200.0, 5.0, 100.0};
    double parsed[PID_PARAMETER_COUNT];
    char *end;
    unsigned int i;
    unsigned int count = 0U;
    const char *cursor;

    if (strcmp(line, "PING") == 0)
        return "PONG\r\n";
    if (strncmp(line, "PID,", 4U) != 0)
        return "ERR,FORMAT\r\n";

    cursor = line + 4;
    for (i = 0U; i < PID_PARAMETER_COUNT; i++)
    {
        parsed[i] = strtod(cursor, &end);
        if (end == cursor)
            return "ERR,FORMAT\r\n";
        count++;
        if (*end == '\0')
            break;
        if (*end != ',' || i == PID_PARAMETER_COUNT - 1U)
            return "ERR,FORMAT\r\n";
        cursor = end + 1;
    }
    if (count != 5U && count != 6U && count != PID_PARAMETER_COUNT)
        return "ERR,FORMAT\r\n";
    for (i = 0U; i < count; i++)
    {
        uint64_t representation;
        /* ARMCC's fast floating-point mode can assume comparisons never see
         * NaN. Inspect the IEEE-754 exponent before comparing user input. */
        memcpy(&representation, &parsed[i], sizeof(representation));
        if ((representation & 0x7ff0000000000000ULL) == 0x7ff0000000000000ULL)
            return "ERR,RANGE\r\n";
        if (!(parsed[i] >= (i == 2U ? -20.0 : -limits[i]) && parsed[i] <= limits[i]))
            return "ERR,RANGE\r\n";
    }
    if ((count >= 6U && parsed[5] != (double)(int)parsed[5]) ||
        (count == PID_PARAMETER_COUNT && parsed[8] != (double)(int)parsed[8]))
        return "ERR,RANGE\r\n";
    for (i = 0U; i < count; i++)
        values[i] = (float)parsed[i];
    return "OK\r\n";
}
