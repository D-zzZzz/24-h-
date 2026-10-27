#ifndef PID_PROTOCOL_H
#define PID_PROTOCOL_H

#define PID_PARAMETER_COUNT 9U

/* Parse one line without CR/LF. Values change only on success.
 * Five/six-value legacy frames leave omitted speed/steering values unchanged. */
const char *PID_ProtocolParse(const char *line, float values[PID_PARAMETER_COUNT]);

#endif
