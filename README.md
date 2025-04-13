# Nortel Millennium VoiceWare Decoder

## 1. Introduction

This command-line utility decodes or lists audio messages stored in Nortel Millennium VoiceWare ROM files. It primarily targets the NEC uPD7759 ADPCM audio format but also handles raw PCM messages by saving their data.

The program parses the specific multi-segment ROM structure (using 0-based indexing), processes ADPCM command streams or raw PCM data, and can output standard PCM WAV files with embedded metadata or list the ROM contents.

## 2. Features

* Decodes NEC uPD7759 ADPCM streams to 16-bit 8kHz mono WAV files.
* Saves raw data for messages identified as Raw PCM (Mode 0x40) to `.pcm` files.
* Handles multi-segment ROM files (concatenated 128KiB segments).
* Uses 0-based indexing for segments and messages within segments.
* Supports an optional mapping file for custom output filenames and comments.
* Provides a listing mode (`-l`, `--list`) to output ROM contents in mapping file format, including alignment and PCM indicators.
* Generates default filenames (`message_S_XXX`) if no mapping is provided (S=0-based segment, XXX=0-based message index).
* Embeds metadata (Album, Artist, Title, Track Number, Date, Comment) into output WAV files.
* Supports verbose (`-v`) and quiet (`-q`) modes.
* Cross-platform compatibility (Linux, macOS, Windows).

## 3. Build Instructions

This project uses CMake.

1.  **Prerequisites:**
    * A C99 compatible C compiler (GCC, Clang, MSVC, etc.)
    * CMake (version 3.10 or later)
    * Build tool (like `make`, `ninja`, or MSBuild/NMake on Windows)

2.  **Build Steps:**
    * Ensure the source file is named `nortel-voiceware-decoder.c` and is in the same directory as `CMakeLists.txt`.
    * Open a terminal or command prompt in the project directory.
    * Run the following commands:

        ```bash
        mkdir build
        cd build
        cmake ..
        # On Linux/macOS with Make (default):
        make
        # Or, using CMake's generic build command:
        # cmake --build .
        # Or, on Windows with MSVC after running cmake:
        # Open the generated .sln file in Visual Studio and build,
        # or use msbuild from the Developer Command Prompt.
        ```
    * The executable `nortel-voiceware-decoder` (or `.exe` on Windows) will be created inside the `build` directory.

## 4. Usage

```bash
./nortel-voiceware-decoder <rom_filepath> [options]

Options:

  <rom_filepath>      Path to the input ROM file. (Required)
  -m <map_filepath>   Path to the optional tab-delimited mapping file.
                      Format: SegIdx(0+)\tMsgIdxInSeg(0+)\tFilenameBase[\tComment]
                      Trailing whitespace is removed from FilenameBase during load.
  -i <message_index>  Decode only the specified absolute message index (0-based).
                      (Ignored if -l or --list is specified).
  -l, --list          List messages in mapping file format (0-based SegIdx) to stdout
                      instead of decoding. Includes header comment '# ROM: <basename>\n\n'.
                      Uses tabs for padding to align comments (assuming 40 char filename width & 8-space tabs).
                      Comments are prefixed with '#'. PCM messages are indicated,
                      avoiding duplication if '(PCM)' is already in map comment.
  -q, --quiet         Quiet mode. Suppress all informational output (stdout & stderr).
                      Only errors are printed to stderr. Overrides -v.
  -v, --verbose       Enable verbose debugging output to stderr. Ignored if -q is used.
  -h, --help          Displays this usage message and exits.
```

## 5. Input File Formats

### 5.1 ROM File

* Binary data dump, potentially multiple 128KiB segments concatenated.
* Each segment starts with a 5-byte header: `last_msg_idx` (u8), `0x5A`, `0xA5`, `0x69`, `0x55`.
* Followed by an offset table of Big-Endian `uint16_t` word offsets to message mode bytes.
* Segments are referenced using **0-based** indices.

### 5.2 Mapping File (Optional, `-m`)

* Plain text, tab-delimited (`\t`).
* Format per line: `` `SegmentIndex\tMessageIndexInSegment\tOutputFilenameBase[\tComment]` ``
* **Important:** `SegmentIndex` and `MessageIndexInSegment` must be **0-based**.
* Lines starting with `#` at the beginning of the line are ignored. Blank lines are ignored.
* Trailing comments (after the optional third tab) have leading `#` and whitespace removed during processing.
* Trailing whitespace is removed from `OutputFilenameBase`.

## 6. Output File Formats

### 6.1 WAV Files (Decode Mode, ADPCM Messages)

* **Naming:** Mapped name or `message_S_XXX.wav`.
* **Format:** Standard RIFF/WAVE, 16-bit PCM, 8000 Hz, Mono.
* **Metadata:** Includes Album, Artist (ROM base name), Title (output base name), Track Number (absolute index), Creation Date, and Comment (from map file, if any).

### 6.2 PCM Files (Decode Mode, PCM Messages)

* **Naming:** Mapped name or `message_S_XXX.pcm`.
* **Format:** Raw binary data copied directly from the ROM, starting with the `0x40` mode byte. Not directly playable as audio.

### 6.3 List Output (List Mode, `-l`)

* Sent to standard output (`stdout`).
* Starts with header: `# ROM: <basename>\n\n`
* Each subsequent line follows the mapping file format, visually aligned:
    `` `SegmentIndex(0+)\tMessageIndexInSegment(0+)\tOutputFilenameBase<tabs...>\t# [ (PCM)][ Comment]\n` ``
* Tab characters are used after `OutputFilenameBase` to visually align the start of the comment field based on a 40-character filename width (assuming 8-space tabs).
* `(PCM)` tag is added if the message mode is `0x40` and the tag wasn't already in the map file comment.

## 7. Known Limitations

* **Raw PCM Mode:** Messages identified with mode byte `0x40` are saved as raw data but not decoded into playable audio.
* **End-of-Prompt:** Relies solely on the `0x00` ADPCM command for message termination.
* **Sample Rate:** Assumes a fixed 8000 Hz sample rate.
* **PCM Scaling:** Uses a `<< 7` bit-shift for ADPCM-to-PCM scaling.

## 8. Related Projects

* [mm_manager](https://github.com/hharte/mm_manager) - Manager for the Nortel Millennium payphone.