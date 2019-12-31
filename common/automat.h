
// 8-Segment VFD Map: 0bABCDEFGH
//
//   DDD
//  F   C
//  F   C
//   EEE
//  G   A
//  G   A
//   HHH  BB
//

const uint8_t VfdUnknownGlyph = 0b00101101;

const uint8_t VfdHexGlyphMap[] = {
// ABCDEFGH
  0b10110111, //   0  "0"
  0b10100000, //   1  "1"
  0b00111011, //   2  "2"
  0b10111001, //   3  "3"
  0b10101100, //   4  "4"
  0b10011101, //   5  "5"
  0b10011111, //   6  "6"
  0b10110000, //   7  "7"
  0b10111111, //   8  "8"
  0b10111101, //   9  "9"
  0b10111110, //  10  "A"
  0b10001111, //  11  "b"
  0b00010111, //  12  "C"
  0b10101011, //  13  "d"
  0b00011111, //  14  "E"
  0b00011110, //  15  "F"
};

const uint8_t VfdAsciiGlyphMap[] = {
// ABCDEFGH
  0b00000000, //   0  NO DISPLAY
  0b00000000, //   1  NO DISPLAY
  0b00000000, //   2  NO DISPLAY
  0b00000000, //   3  NO DISPLAY
  0b00000000, //   4  NO DISPLAY
  0b00000000, //   5  NO DISPLAY
  0b00000000, //   6  NO DISPLAY
  0b00000000, //   7  NO DISPLAY
  0b00000000, //   8  NO DISPLAY
  0b00000000, //   9  NO DISPLAY
  0b00000000, //  10  NO DISPLAY
  0b00000000, //  11  NO DISPLAY
  0b00000000, //  12  NO DISPLAY
  0b00000000, //  13  NO DISPLAY
  0b00000000, //  14  NO DISPLAY
  0b00000000, //  15  NO DISPLAY
  0b00000000, //  16  NO DISPLAY
  0b00000000, //  17  NO DISPLAY
  0b00000000, //  18  NO DISPLAY
  0b00000000, //  19  NO DISPLAY
  0b00000000, //  20  NO DISPLAY
  0b00000000, //  21  NO DISPLAY
  0b00000000, //  22  NO DISPLAY
  0b00000000, //  23  NO DISPLAY
  0b00000000, //  24  NO DISPLAY
  0b00000000, //  25  NO DISPLAY
  0b00000000, //  26  NO DISPLAY
  0b00000000, //  27  NO DISPLAY
  0b00000000, //  28  NO DISPLAY
  0b00000000, //  29  NO DISPLAY
  0b00000000, //  30  NO DISPLAY
  0b00000000, //  31  NO DISPLAY
  0b00000000, //  32  ' '
  0b01100000, //  33  '!'
  0b00100100, //  34  '"'
  0b00111100, //  35  '#'
  0b00000000, //  36  '$'  NO DISPLAY
  0b10000100, //  37  '%'
  0b00011001, //  38  '&'
  0b00100000, //  39  '''
  0b00010111, //  40  '('
  0b10110001, //  41  ')'
  0b01001000, //  42  '*'
  0b00001110, //  43  '+'
  0b10000001, //  44  ','
  0b00001000, //  45  '-'
  0b01000000, //  46  '.'
  0b00101010, //  47  '/'
  0b10110111, //  48  "0"
  0b10100000, //  49  "1"
  0b00111011, //  50  "2"
  0b10111001, //  51  "3"
  0b10101100, //  52  "4"
  0b10011101, //  53  "5"
  0b10011111, //  54  "6"
  0b10110000, //  55  "7"
  0b10111111, //  56  "8"
  0b10111101, //  57  "9"
  0b00010001, //  58  ':'
  0b10010000, //  59  ';'
  0b00001100, //  60  '<'
  0b00001001, //  61  '='
  0b00101000, //  62  '>'
  0b01110000, //  63  '?'
  0b10011011, //  64  '@'
  0b10111110, //  65  "A"
  0b10001111, //  66  "b"
  0b00010111, //  67  "C"
  0b10101011, //  68  "d"
  0b00011111, //  69  "E"
  0b00011110, //  70  "F"
  0b10010111, //  71  'G'
  0b10101110, //  72  'H'
  0b10100000, //  73  'I'
  0b10100011, //  74  'J'
  0b01101110, //  75  'k'
  0b00000111, //  76  'L'
  0b00100100, //  77  'm'
  0b10001010, //  78  'n'
  0b10110111, //  79  'O'
  0b00111110, //  80  'P'
  0b10111100, //  81  'q'
  0b00001010, //  82  'r'
  0b10011101, //  83  's'
  0b00001111, //  84  't'
  0b10100111, //  85  'U'
  0b10100011, //  86  'V'
  0b00100101, //  87  'w'
  0b00100010, //  88  'X'
  0b10101101, //  89  'Y'
  0b00110011, //  90  'Z'
  0b00010111, //  91  '['
  0b10001100, //  92  '\'
  0b10110001, //  93  ']'
  0b00110100, //  94  '^'
  0b00000001, //  95  '_'
  0b00000100, //  96  '`'
  0b10111110, //  97  "A"
  0b10001111, //  98  "b"
  0b00001011, //  99  "c"
  0b10101011, // 100  "d"
  0b00111111, // 101  "e"
  0b00011110, // 102  "F"
  0b10010111, // 103  'G'
  0b10001110, // 104  'h'
  0b10000000, // 105  'i'
  0b10100011, // 106  'j'
  0b01101110, // 107  'k'
  0b10100000, // 108  'l'
  0b00100100, // 109  'm'
  0b10001010, // 110  'n'
  0b10001011, // 111  'o'
  0b00111110, // 112  'P'
  0b10111100, // 113  'q'
  0b00001010, // 114  'r'
  0b10011101, // 115  's'
  0b00001111, // 116  't'
  0b10000011, // 117  'u'
  0b10000001, // 118  'v'
  0b00100101, // 119  'w'
  0b00100010, // 120  'X'
  0b10101101, // 121  'Y'
  0b00110011, // 122  'Z'
  0b00000000, // 123  '0b' NO DISPLAY
  0b00000110, // 124  '|'
  0b10000000, // 125  ','
  0b00001000, // 126  '~'
  0b00000000, // 127  'DEL' NO DISPLAY
};

uint8_t vfd_render_glyph(
    const uint8_t glyph_map[],
    size_t glyph_map_size,
    uint8_t unknown_glyph,
    uint8_t source) {
  uint8_t glyph = unknown_glyph;
  if (source < glyph_map_size) {
    glyph = glyph_map[source];
  }
  return glyph;
}

void vfd_render_glyphs(
    const uint8_t glyph_map[],
    size_t glyph_map_size,
    uint8_t unknown_glyph,
    uint8_t target[],
    const uint8_t source[],
    size_t length) {
  for (int i = 0; i < length; ++i) {
    target[i] = vfd_render_glyph(
      glyph_map,
      glyph_map_size,
      unknown_glyph,
      source[i]);
  }
}

uint8_t vfd_render_ascii_glyph(uint8_t source) {
  return vfd_render_glyph(
    VfdAsciiGlyphMap,
    sizeof(VfdAsciiGlyphMap),
    VfdUnknownGlyph,
    source);
}

void vfd_render_ascii_buffer(
    uint8_t target[],
    const byte source[],
    size_t length) {
  vfd_render_glyphs(
    VfdAsciiGlyphMap,
    sizeof(VfdAsciiGlyphMap),
    VfdUnknownGlyph,
    target,
    source,
    length);
}

void vfd_render_ascii_string(
  uint8_t target[],
  const String source) {

  vfd_render_ascii_buffer(
    target,
    (const byte *) source.c_str(),
    source.length());
}

