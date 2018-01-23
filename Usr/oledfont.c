#include "oledhzfont.h"

char Hzk[][32]={

{0x00,0x00,0xF0,0x10,0x10,0x10,0x10,0xFF,0x10,0x10,0x10,0x10,0xF0,0x00,0x00,0x00},
{0x00,0x00,0x0F,0x04,0x04,0x04,0x04,0xFF,0x04,0x04,0x04,0x04,0x0F,0x00,0x00,0x00},/*"中",0*/

{0x40,0x40,0x40,0x5F,0x55,0x55,0x55,0x75,0x55,0x55,0x55,0x5F,0x40,0x40,0x40,0x00},
{0x00,0x40,0x20,0x0F,0x09,0x49,0x89,0x79,0x09,0x09,0x09,0x0F,0x20,0x40,0x00,0x00},/*"景",1*/

{0x00,0xFE,0x02,0x42,0x4A,0xCA,0x4A,0x4A,0xCA,0x4A,0x4A,0x42,0x02,0xFE,0x00,0x00},
{0x00,0xFF,0x40,0x50,0x4C,0x43,0x40,0x40,0x4F,0x50,0x50,0x5C,0x40,0xFF,0x00,0x00},/*"园",2*/

{0x00,0x00,0xF8,0x88,0x88,0x88,0x88,0xFF,0x88,0x88,0x88,0x88,0xF8,0x00,0x00,0x00},
{0x00,0x00,0x1F,0x08,0x08,0x08,0x08,0x7F,0x88,0x88,0x88,0x88,0x9F,0x80,0xF0,0x00},/*"电",3*/

{0x80,0x82,0x82,0x82,0x82,0x82,0x82,0xE2,0xA2,0x92,0x8A,0x86,0x82,0x80,0x80,0x00},
{0x00,0x00,0x00,0x00,0x00,0x40,0x80,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*"子",4*/

{0x24,0x24,0xA4,0xFE,0xA3,0x22,0x00,0x22,0xCC,0x00,0x00,0xFF,0x00,0x00,0x00,0x00},
{0x08,0x06,0x01,0xFF,0x00,0x01,0x04,0x04,0x04,0x04,0x04,0xFF,0x02,0x02,0x02,0x00},/*"科",5*/

{0x10,0x10,0x10,0xFF,0x10,0x90,0x08,0x88,0x88,0x88,0xFF,0x88,0x88,0x88,0x08,0x00},
{0x04,0x44,0x82,0x7F,0x01,0x80,0x80,0x40,0x43,0x2C,0x10,0x28,0x46,0x81,0x80,0x00},/*"技",6*/

};

char HzSTART[][32]={
{0x80,0x82,0x82,0x82,0xFE,0x82,0x82,0x82,0x82,0x82,0xFE,0x82,0x82,0x82,0x80,0x00},
{0x00,0x80,0x40,0x30,0x0F,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00},/*"开",0*/

{0x10,0x10,0xF0,0x1F,0x10,0xF0,0x00,0x40,0xE0,0x58,0x47,0x40,0x50,0x60,0xC0,0x00},
{0x40,0x22,0x15,0x08,0x16,0x21,0x00,0x00,0xFE,0x42,0x42,0x42,0x42,0xFE,0x00,0x00},/*"始",1*/
};

char HzSTOP[][32]={
{0x80,0x60,0xF8,0x07,0x00,0x04,0x74,0x54,0x55,0x56,0x54,0x54,0x74,0x04,0x00,0x00},
{0x00,0x00,0xFF,0x00,0x03,0x01,0x05,0x45,0x85,0x7D,0x05,0x05,0x05,0x01,0x03,0x00},/*"停",0*/

{0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0xFF,0x40,0x40,0x40,0x40,0x40,0x00,0x00,0x00},
{0x40,0x40,0x40,0x7F,0x40,0x40,0x40,0x7F,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00},/*"止",1*/
};

char HzTask[][32]={
{0x00,0x80,0x60,0xF8,0x87,0x80,0x84,0x84,0x84,0xFE,0x82,0x83,0x82,0x80,0x80,0x00},
{0x01,0x00,0x00,0xFF,0x00,0x40,0x40,0x40,0x40,0x7F,0x40,0x40,0x40,0x40,0x00,0x00},/*任,0*/

{0x00,0x00,0x90,0x88,0x4C,0x57,0xA4,0x24,0x54,0x54,0x8C,0x84,0x00,0x00,0x00,0x00},
{0x01,0x01,0x80,0x42,0x22,0x1A,0x07,0x02,0x42,0x82,0x42,0x3E,0x01,0x01,0x01,0x00},/*务,1*/
};
