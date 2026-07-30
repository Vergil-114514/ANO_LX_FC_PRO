#include "Drv_UwbMini5.h"
#include "SysConfig.h"
#include "Ano_Math.h"
#include "Drv_Sys.h"
#include <string.h>

#define UWB_LINE_BUF_SIZE 128U
#define UWB_ANCHOR_COUNT 4U

_uwb_mini5_st UwbMini5;

static char uwbLineBuf[UWB_LINE_BUF_SIZE];
static uint8_t uwbLineLen;
static uint8_t uwbLineOverflow;

static const int32_t uwbAnchorX[UWB_ANCHOR_COUNT] =
{
    UWB_A0_X_MM, UWB_A1_X_MM, UWB_A2_X_MM, UWB_A3_X_MM
};

static const int32_t uwbAnchorY[UWB_ANCHOR_COUNT] =
{
    UWB_A0_Y_MM, UWB_A1_Y_MM, UWB_A2_Y_MM, UWB_A3_Y_MM
};

static uint8_t uwbIsSeparator(const char data)
{
    return (data == ' ') || (data == '\t') || (data == ',');
}

static uint8_t uwbNextToken(const char **cursor, const char **start, uint8_t *length)
{
    const char *p = *cursor;

    while ((*p != '\0') && uwbIsSeparator(*p))
    {
        p++;
    }

    if (*p == '\0')
    {
        return 0U;
    }

    *start = p;
    while ((*p != '\0') && !uwbIsSeparator(*p))
    {
        p++;
    }

    *length = (uint8_t)(p - *start);
    *cursor = p;
    return 1U;
}

static uint8_t uwbTokenEquals(const char *token, const uint8_t length, const char *text)
{
    uint8_t index = 0U;

    while (text[index] != '\0')
    {
        if ((index >= length) || (token[index] != text[index]))
        {
            return 0U;
        }
        index++;
    }

    return index == length;
}

static uint8_t uwbParseHex(const char *token, uint8_t length, uint32_t *value)
{
    uint32_t result = 0U;
    uint8_t digitCount = 0U;
    uint8_t digit;

    if ((length >= 2U) && (token[0] == '0') && ((token[1] == 'x') || (token[1] == 'X')))
    {
        token += 2;
        length -= 2U;
    }

    while (length-- > 0U)
    {
        if ((*token >= '0') && (*token <= '9'))
        {
            digit = (uint8_t)(*token - '0');
        }
        else if ((*token >= 'a') && (*token <= 'f'))
        {
            digit = (uint8_t)(*token - 'a' + 10U);
        }
        else if ((*token >= 'A') && (*token <= 'F'))
        {
            digit = (uint8_t)(*token - 'A' + 10U);
        }
        else
        {
            return 0U;
        }

        if (result > 0x0FFFFFFFU)
        {
            return 0U;
        }

        result = (result << 4) | digit;
        digitCount++;
        token++;
    }

    if (digitCount == 0U)
    {
        return 0U;
    }

    *value = result;
    return 1U;
}

static uint8_t uwbParseEndpoint(const char *token, const uint8_t length, uint8_t *tagId)
{
    uint8_t separator = 1U;
    uint32_t value;

    if ((length < 4U) ||
        ((token[0] != 'a') && (token[0] != 'A') &&
         (token[0] != 't') && (token[0] != 'T')))
    {
        return 0U;
    }

    while ((separator < length) && (token[separator] != ':'))
    {
        separator++;
    }

    if ((separator == 1U) || (separator >= (uint8_t)(length - 1U)))
    {
        return 0U;
    }

    if (!uwbParseHex(&token[1], (uint8_t)(separator - 1U), &value) || (value > 0xFFU))
    {
        return 0U;
    }

    *tagId = (uint8_t)value;
    return 1U;
}

static uint8_t uwbSolvePosition(const uint32_t rangeMm[UWB_ANCHOR_COUNT], int32_t *positionX, int32_t *positionY, uint32_t *residualMm)
{
    float normalAA = 0.0f;
    float normalAB = 0.0f;
    float normalBB = 0.0f;
    float normalAC = 0.0f;
    float normalBC = 0.0f;
    float positionXf;
    float positionYf;
    float determinant;
    float heightSquare = 0.0f;
    float residualSquare = 0.0f;
    uint8_t anchor;

    for (anchor = 1U; anchor < UWB_ANCHOR_COUNT; anchor++)
    {
        int64_t dx = (int64_t)uwbAnchorX[anchor] - uwbAnchorX[0];
        int64_t dy = (int64_t)uwbAnchorY[anchor] - uwbAnchorY[0];
        int64_t coordinateSquare = (int64_t)uwbAnchorX[anchor] * uwbAnchorX[anchor] + (int64_t)uwbAnchorY[anchor] * uwbAnchorY[anchor] - (int64_t)uwbAnchorX[0] * uwbAnchorX[0] - (int64_t)uwbAnchorY[0] * uwbAnchorY[0];
        int64_t rangeSquare = (int64_t)rangeMm[anchor] * rangeMm[anchor] - (int64_t)rangeMm[0] * rangeMm[0];
        float equationA = (float)(2 * dx);
        float equationB = (float)(2 * dy);
        float equationC = (float)(coordinateSquare - rangeSquare);

        normalAA += equationA * equationA;
        normalAB += equationA * equationB;
        normalBB += equationB * equationB;
        normalAC += equationA * equationC;
        normalBC += equationB * equationC;
    }

    determinant = normalAA * normalBB - normalAB * normalAB;
    if ((determinant > -0.001f) && (determinant < 0.001f))
    {
        return 0U;
    }

    positionXf = (normalAC * normalBB - normalBC * normalAB) / determinant;
    positionYf = (normalBC * normalAA - normalAC * normalAB) / determinant;

    for (anchor = 0U; anchor < UWB_ANCHOR_COUNT; anchor++)
    {
        float dx = positionXf - (float)uwbAnchorX[anchor];
        float dy = positionYf - (float)uwbAnchorY[anchor];
        heightSquare += (float)rangeMm[anchor] * rangeMm[anchor] - dx * dx - dy * dy;
    }
    heightSquare /= UWB_ANCHOR_COUNT;

    if (heightSquare < 0.0f)
    {
        heightSquare = 0.0f;
    }

    for (anchor = 0U; anchor < UWB_ANCHOR_COUNT; anchor++)
    {
        float dx = positionXf - (float)uwbAnchorX[anchor];
        float dy = positionYf - (float)uwbAnchorY[anchor];
        float calculatedRange = my_sqrt(dx * dx + dy * dy + heightSquare);
        float rangeError = calculatedRange - (float)rangeMm[anchor];
        residualSquare += rangeError * rangeError;
    }

    *positionX = (int32_t)(positionXf + ((positionXf >= 0.0f) ? 0.5f : -0.5f));
    *positionY = (int32_t)(positionYf + ((positionYf >= 0.0f) ? 0.5f : -0.5f));
    *residualMm = (uint32_t)(my_sqrt(residualSquare / UWB_ANCHOR_COUNT) + 0.5f);
    return 1U;
}

static void uwbParseLine(const char *line)
{
    const char *token[10];
    uint8_t length[10];
    const char *cursor = line;
    uint32_t mask;
    uint32_t rangeMm[UWB_ANCHOR_COUNT];
    uint32_t rseq;
    uint32_t nranges;
    uint32_t debug;
    uint8_t tagId;
    uint8_t index;
    int32_t positionX;
    int32_t positionY;
    uint32_t residualMm;

    for (index = 0U; index < 10U; index++)
    {
        if (!uwbNextToken(&cursor, &token[index], &length[index]))
        {
            UwbMini5.rejected_frame_count++;
            return;
        }
    }

    if (!uwbTokenEquals(token[0], length[0], "mc") ||
        !uwbParseHex(token[1], length[1], &mask) ||
        !uwbParseHex(token[6], length[6], &nranges) ||
        !uwbParseHex(token[7], length[7], &rseq) ||
        !uwbParseHex(token[8], length[8], &debug) ||
        !uwbParseEndpoint(token[9], length[9], &tagId) ||
        (tagId != UWB_TAG_SHORT_ID) || (rseq > 0xFFU))
    {
        UwbMini5.rejected_frame_count++;
        return;
    }

    for (index = 0U; index < UWB_ANCHOR_COUNT; index++)
    {
        if (!uwbParseHex(token[index + 2U], length[index + 2U], &rangeMm[index]) ||
            (rangeMm[index] < UWB_RANGE_MIN_MM) || (rangeMm[index] > UWB_RANGE_MAX_MM))
        {
            UwbMini5.rejected_frame_count++;
            return;
        }
    }

    UwbMini5.anchor_mask = (uint8_t)mask;
    if (mask != 0x0FU)
    {
        UwbMini5.rejected_frame_count++;
        return;
    }

    if (UwbMini5.rseq_valid != 0U)
    {
        uint8_t delta = (uint8_t)((uint8_t)rseq - UwbMini5.last_rseq);
        if ((delta == 0U) || (delta >= 128U))
        {
            UwbMini5.rejected_frame_count++;
            return;
        }
    }

    if (!uwbSolvePosition(rangeMm, &positionX, &positionY, &residualMm))
    {
        UwbMini5.rejected_frame_count++;
        return;
    }

    UwbMini5.position_mm[0] = positionX;
    UwbMini5.position_mm[1] = positionY;
    UwbMini5.position_mm[2] = 0;
    UwbMini5.residual_mm = residualMm;
    UwbMini5.last_update_ms = GetSysRunTimeMs();
    UwbMini5.age_ms = 0U;
    UwbMini5.valid = 1U;
    UwbMini5.last_rseq = (uint8_t)rseq;
    UwbMini5.rseq_valid = 1U;
    UwbMini5.position_update_cnt++;
    UwbMini5.valid_frame_count++;
}

void DrvUwbMini5Init(void)
{
    memset(&UwbMini5, 0, sizeof(UwbMini5));
    uwbLineLen = 0U;
    uwbLineOverflow = 0U;
}

void DrvUwbMini5RxOneByte(const uint8_t linktype, const uint8_t data)
{
    (void)linktype;

    if (data == '\r')
    {
        return;
    }

    if (data == '\n')
    {
        if (uwbLineOverflow != 0U)
        {
            UwbMini5.rejected_frame_count++;
        }
        else if (uwbLineLen > 0U)
        {
            uwbLineBuf[uwbLineLen] = '\0';
            uwbParseLine(uwbLineBuf);
        }

        uwbLineLen = 0U;
        uwbLineOverflow = 0U;
        return;
    }

    if (uwbLineOverflow != 0U)
    {
        return;
    }

    if (uwbLineLen >= (UWB_LINE_BUF_SIZE - 1U))
    {
        uwbLineOverflow = 1U;
        return;
    }

    uwbLineBuf[uwbLineLen++] = (char)data;
}

void DrvUwbMini5Task1ms(void)
{
    if (UwbMini5.valid == 0U)
    {
        return;
    }

    if (UwbMini5.age_ms < 0xFFFFU)
    {
        UwbMini5.age_ms++;
    }

    if (UwbMini5.age_ms > UWB_DATA_TIMEOUT_MS)
    {
        UwbMini5.valid = 0U;
        UwbMini5.rseq_valid = 0U;
    }
}
