/**
 * @file nortel-voiceware-decoder.c
 * @brief Decodes Nortel Millennium VoiceWare ROM files (NEC uPD7759 ADPCM).
 *
 * This command-line utility decodes audio messages from Nortel Millennium
 * VoiceWare ROM dumps, primarily handling NEC uPD7759 ADPCM encoded messages.
 * It parses the ROM structure, decodes ADPCM command streams, and outputs
 * standard PCM WAV files with embedded metadata. It can also list the
 * contents of the ROM in mapping file format. Uses 0-based segment indexing.
 *
 * Build with CMake:
 * mkdir build
 * cd build
 * cmake ..
 * make
 *
 * Usage:
 * ./nortel-voiceware-decoder <rom_filepath> [-m <map_filepath>] [-i <message_index>] [-l|--list] [-q|--quiet] [-v|--verbose]
 *
 * Options:
 * <rom_filepath>      : Path to the input ROM file.
 * -m <map_filepath>   : Path to the optional tab-delimited mapping file.
 *			 Format: SegIdx(0+)\tMsgIdxInSeg(0+)\tFilenameBase[\tComment]
 *			 Trailing whitespace is removed from FilenameBase during load.
 * -i <message_index>  : Decode only the specified absolute message index (0-based). Ignored if -l is used.
 * -l, --list          : List messages in mapping file format (0-based SegIdx) to stdout instead of decoding.
 *			 Output includes a header comment '# ROM: <basename>\n\n'.
 *			 Uses tabs for padding to align comments (assuming 40 char filename width & 8-space tabs).
 *			 Comments are prefixed with '#'. PCM messages are indicated,
 *			 avoiding duplication if '(PCM)' is already in map comment.
 * -q, --quiet         : Quiet mode. Suppress all informational output (stdout & stderr). Only errors are printed to stderr. Overrides -v.
 * -v, --verbose       : Enable verbose debugging output to stderr. Ignored if -q is used.
 */

 #define _CRT_SECURE_NO_WARNINGS /* Disable security warnings for fopen, etc. on MSVC */
 #define _DEFAULT_SOURCE         /* Enable strdup on Linux/POSIX */

 #include <stdio.h>
 #include <stdlib.h>
 #include <stdint.h>
 #include <string.h>
 #include <stdbool.h>
 #include <time.h>
 #include <ctype.h> /* For isspace */
 #include <limits.h> /* For UINT32_MAX */
 #include <stdarg.h> /* For va_list */

 #ifdef _MSC_VER
 #pragma warning(disable : 4996) /* Disable deprecation warnings for fopen, etc. */
 #pragma warning(disable : 5045) /* Disable Spectre mitigation warning */
 #include <BaseTsd.h>
 typedef SSIZE_T ssize_t; /* Define ssize_t for MSVC */
 #define strdup _strdup     /* Use _strdup on MSVC */
 #else
 #include <unistd.h> /* For getopt (if used) */
 #endif

 /* --- Build Info Defines (Defaults for local builds) --- */
 #ifndef GIT_COMMIT_HASH
 #define GIT_COMMIT_HASH "local"
 #endif
 #ifndef GIT_TAG_NAME
 #define GIT_TAG_NAME "local"
 #endif


 /* --- Constants --- */
 #define ROM_SEGMENT_SIZE 131072 /* 128 KiB */
 #define MAX_MESSAGES_PER_SEGMENT 256 /* Based on uint8_t index */
 #define DEFAULT_SAMPLE_RATE 8000
 #define ADPCM_BITS 16 /* Output PCM bits */
 #define ADPCM_CHANNELS 1 /* Mono */
 #define LIST_FILENAME_ALIGN_WIDTH 40 /* Width for filename alignment in list mode */
 #define TAB_WIDTH 8 /* Assumed tab width for alignment calculation */


 /* ROM Header Magic Number */
 const uint8_t ROM_MAGIC[4] = {0x5A, 0xA5, 0x69, 0x55};

 /* Message Modes */
 #define MODE_ADPCM 0x00
 #define MODE_PCM 0x40 /* Detected but not fully decoded */

 /* ADPCM Decoding Tables (New) */
 /* Step size adjustment table (Delta values) - Now 2D */
 static const int step_table[16][16] = {
     { 0, 0, 1, 2, 3, 5, 7, 10, 0, 0, -1, -2, -3, -5, -7, -10 }, { 0, 1, 2, 3, 4, 6, 8, 13, 0, -1, -2, -3, -4, -6, -8, -13 },
     { 0, 1, 2, 4, 5, 7, 10, 15, 0, -1, -2, -4, -5, -7, -10, -15 }, { 0, 1, 3, 4, 6, 9, 13, 19, 0, -1, -3, -4, -6, -9, -13, -19 },
     { 0, 2, 3, 5, 8, 11, 15, 23, 0, -2, -3, -5, -8, -11, -15, -23 }, { 0, 2, 4, 7, 10, 14, 19, 29, 0, -2, -4, -7, -10, -14, -19, -29 },
     { 0, 3, 5, 8, 12, 16, 22, 33, 0, -3, -5, -8, -12, -16, -22, -33 }, { 1, 4, 7, 10, 15, 20, 29, 43, -1, -4, -7, -10, -15, -20, -29, -43 },
     { 1, 4, 8, 13, 18, 25, 35, 53, -1, -4, -8, -13, -18, -25, -35, -53 }, { 1, 6, 10, 16, 22, 31, 43, 64, -1, -6, -10, -16, -22, -31, -43, -64 },
     { 2, 7, 12, 19, 27, 37, 51, 76, -2, -7, -12, -19, -27, -37, -51, -76 }, { 2, 9, 16, 24, 34, 46, 64, 96, -2, -9, -16, -24, -34, -46, -64, -96 },
     { 3, 11, 19, 29, 41, 57, 79, 117, -3, -11, -19, -29, -41, -57, -79, -117 }, { 4, 13, 24, 36, 50, 69, 96, 143, -4, -13, -24, -36, -50, -69, -96, -143 },
     { 4, 16, 29, 44, 62, 85, 118, 175, -4, -16, -29, -44, -62, -85, -118, -175 }, { 6, 20, 36, 54, 76, 104, 144, 214, -6, -20, -36, -54, -76, -104, -144, -214 },
 };

 /* State Adjustment Table (New) */
 static const int state_table[16] = { -1, -1, 0, 0, 1, 2, 2, 3, -1, -1, 0, 0, 1, 2, 2, 3 };


 /* --- Global Variables --- */
 bool verbose_mode = false;
 bool list_mode = false; /* Flag for listing mode */
 bool quiet_mode = false; /* Flag for quiet mode */

 /* --- Data Structures (Moved Before Forward Declarations) --- */

 /**
  * struct message_mapping - Holds information parsed from the mapping file.
  * @segment_index:         0-based segment index from map file.
  * @message_index_in_seg:  0-based message index within segment from map file.
  * @output_filename_base:  Malloc'd base filename (no extension).
  * @comment:               Malloc'd comment string (cleaned).
  */
 typedef struct {
     int segment_index;
     int message_index_in_seg;
     char *output_filename_base;
     char *comment;
 } MessageMapping;

 /**
  * struct mapping_table - Dynamic array to store message mappings.
  * @mappings: Pointer to array of MessageMapping structs.
  * @count:    Number of mappings currently stored.
  * @capacity: Allocated capacity of the mappings array.
  */
 typedef struct {
     MessageMapping *mappings;
     size_t count;
     size_t capacity;
 } MappingTable;

 /**
  * struct adpcm_state - Holds the state for the ADPCM decoder.
  * @current_sample: Current predicted sample.
  * @adpcm_state:    Current state index (0-15).
  */
 typedef struct {
     int16_t current_sample;
     int8_t adpcm_state; /* State index remains 0-15 */
 } AdpcmState;

 /**
  * struct pcm_buffer - Dynamic buffer for storing decoded PCM samples.
  * @samples:  Pointer to array of 16-bit PCM samples.
  * @count:    Number of samples currently stored.
  * @capacity: Allocated capacity in samples.
  */
 typedef struct {
     int16_t *samples;
     size_t count;
     size_t capacity;
 } PcmBuffer;

 /**
  * enum handle_message_result - Return codes for handle_message_iteration.
  * @MSG_HANDLED_CONTINUE:      Processing successful, continue loop.
  * @MSG_HANDLED_TARGET_FOUND:  Target message processed successfully (decode mode).
  * @MSG_HANDLED_ERROR:         An error occurred during processing/listing.
  */
 typedef enum {
     MSG_HANDLED_CONTINUE,
     MSG_HANDLED_TARGET_FOUND,
     MSG_HANDLED_ERROR
 } HandleMessageResult;


 /* --- Forward Declarations --- */
 void print_usage(const char *prog_name);
 bool process_message(const uint8_t *rom_data, size_t rom_size,
         size_t segment_start_offset, int segment_index_0_based,
         int msg_idx_in_segment, int absolute_msg_idx,
         uint32_t message_offset_in_segment, uint32_t next_message_offset_in_segment,
         const MessageMapping *mapping, const char *rom_basename);
 HandleMessageResult handle_message_iteration(
     const uint8_t *rom_data, size_t rom_size,
     size_t segment_start_offset, int segment_index_0_based,
     uint32_t msg_idx_in_seg, int absolute_msg_idx,
     const uint16_t *offset_table, uint32_t message_count_in_segment,
     const MappingTable *mapping_table, const char *rom_basename,
     bool list_mode, bool quiet_mode, long target_message_idx);
 bool load_mappings(const char *filepath, MappingTable *table); /* Needed by load_mapping_data */


 /* --- Utility Functions --- */

 /**
  * status_printf() - Prints status messages to stdout unless quiet_mode enabled.
  * @format: Printf-style format string.
  * @...:    Arguments for the format string.
  */
 void
 status_printf(const char *format, ...)
 {
     if (!quiet_mode) {
         va_list args;
         va_start(args, format);
         vprintf(format, args);
         va_end(args);
     }
 }


 /**
  * verbose_printf() - Prints verbose messages to stderr if verbose_mode enabled.
  * @format: Printf-style format string.
  * @...:    Arguments for the format string.
  */
 void
 verbose_printf(const char *format, ...)
 {
     /* Note: quiet_mode check is implicit as it forces verbose_mode off */
     if (verbose_mode) {
         va_list args;
         va_start(args, format);
         vfprintf(stderr, format, args);
         va_end(args);
     }
 }

 /**
  * read_u16be() - Reads a 16-bit unsigned integer in Big-Endian format.
  * @buffer: Pointer to the buffer.
  *
  * Return: The uint16_t value.
  */
 uint16_t
 read_u16be(const uint8_t *buffer)
 {
     return ((uint16_t)buffer[0] << 8) | buffer[1];
 }

 /**
  * write_u16le() - Writes a 16-bit unsigned integer in Little-Endian format.
  * @value: The value to write.
  * @fp:    File pointer.
  *
  * Return: true on success, false on failure.
  */
 bool
 write_u16le(uint16_t value, FILE *fp)
 {
     uint8_t buffer[2];

     buffer[0] = value & 0xFF;
     buffer[1] = (value >> 8) & 0xFF;
     return fwrite(buffer, 1, 2, fp) == 2;
 }

 /**
  * write_u32le() - Writes a 32-bit unsigned integer in Little-Endian format.
  * @value: The value to write.
  * @fp:    File pointer.
  *
  * Return: true on success, false on failure.
  */
 bool
 write_u32le(uint32_t value, FILE *fp)
 {
     uint8_t buffer[4];

     buffer[0] = value & 0xFF;
     buffer[1] = (value >> 8) & 0xFF;
     buffer[2] = (value >> 16) & 0xFF;
     buffer[3] = (value >> 24) & 0xFF;
     return fwrite(buffer, 1, 4, fp) == 4;
 }

 /**
  * write_chunk_id() - Writes a 4-character chunk ID to a file.
  * @id: The 4-character string ID.
  * @fp: File pointer.
  *
  * Return: true on success, false on failure.
  */
 bool
 write_chunk_id(const char *id, FILE *fp)
 {
     return fwrite(id, 1, 4, fp) == 4;
 }

 /**
  * get_base_filename() - Extracts the base filename from a full path.
  * @filepath: The full path string.
  *
  * Return: A pointer to the start of the base filename within the input string.
  */
 const char *
 get_base_filename(const char *filepath)
 {
     const char *last_slash = strrchr(filepath, '/');
     const char *last_bslash = strrchr(filepath, '\\');
     const char *base = filepath;

     if (last_slash && last_slash >= base)
         base = last_slash + 1;

     if (last_bslash && last_bslash >= base)
         base = last_bslash + 1;

     return base;
 }

 /**
  * clean_comment() - Cleans comment string by removing leading whitespace,
  * the first '#' encountered after that whitespace (if any),
  * and any whitespace immediately following that '#'.
  * @comment: The comment string to clean (modified in place).
  */
 void
 clean_comment(char *comment)
 {
     char *start_of_text = comment;

     if (!comment || *comment == '\0')
         return;

     /* 1. Find the first non-whitespace character */
     while (*start_of_text != '\0' && isspace((unsigned char)*start_of_text))
         start_of_text++;

     /* 2. Check if it's a hash */
     if (*start_of_text == '#') {
         /* 3. If yes, find the first character after the hash and subsequent whitespace */
         start_of_text++; /* Move past '#' */
         while (*start_of_text != '\0' && isspace((unsigned char)*start_of_text))
             start_of_text++;
     }
     /* If it wasn't a hash, start_of_text already points to the first non-whitespace char */

     /* 4. Shift the string if start_of_text moved from the original start */
     if (start_of_text != comment)
         memmove(comment, start_of_text, strlen(start_of_text) + 1);
 }


 /* --- Mapping File Handling --- */

 /**
  * init_mapping_table() - Initializes a MappingTable.
  * @table: Pointer to the MappingTable.
  */
 void
 init_mapping_table(MappingTable *table)
 {
     table->mappings = NULL;
     table->count = 0;
     table->capacity = 0;
 }

 /**
  * add_mapping() - Adds a mapping entry to the table, handling duplicates.
  * @table: Pointer to the MappingTable.
  * @entry: The MessageMapping entry to add (ownership of strings transferred).
  *
  * Return: true on success, false on memory allocation failure.
  */
 bool
 add_mapping(MappingTable *table, MessageMapping entry)
 {
     size_t i;

     /* Check for duplicates and replace if found */
     for (i = 0; i < table->count; ++i) {
         /* Compare using 0-based indices */
         if (table->mappings[i].segment_index == entry.segment_index &&
             table->mappings[i].message_index_in_seg == entry.message_index_in_seg) {
             verbose_printf("Replacing duplicate mapping for Segment %d, Message %d\n",
                        entry.segment_index, entry.message_index_in_seg);
             free(table->mappings[i].output_filename_base);
             free(table->mappings[i].comment);
             table->mappings[i] = entry; /* Replace existing entry */
             return true;
         }
     }

     /* Grow capacity if needed */
     if (table->count >= table->capacity) {
         size_t new_capacity = (table->capacity == 0) ? 16 : table->capacity * 2;
         MessageMapping *new_mappings = (MessageMapping *)realloc(table->mappings, new_capacity * sizeof(MessageMapping));
         if (!new_mappings) {
             fprintf(stderr, "ERROR: Failed to allocate memory for mapping table.\n");
             /* Free the passed-in entry's strings as it wasn't added */
             free(entry.output_filename_base);
             free(entry.comment);
             return false;
         }
         table->mappings = new_mappings;
         table->capacity = new_capacity;
     }

     /* Add the new entry */
     table->mappings[table->count++] = entry;
     return true;
 }

 /**
  * free_mapping_table() - Frees memory associated with a MappingTable.
  * @table: Pointer to the MappingTable.
  */
 void
 free_mapping_table(MappingTable *table)
 {
     size_t i;

     if (table && table->mappings) {
         for (i = 0; i < table->count; ++i) {
             free(table->mappings[i].output_filename_base);
             free(table->mappings[i].comment);
         }
         free(table->mappings);
     }
     table->mappings = NULL;
     table->count = 0;
     table->capacity = 0;
 }

 /**
  * load_mappings() - Loads and parses the mapping file (expects 0-based segment index).
  * @filepath: Path to the mapping file.
  * @table:    Pointer to the MappingTable to populate.
  *
  * Return: true on success, false on failure.
  */
 bool
 load_mappings(const char *filepath, MappingTable *table)
 {
     FILE *fp;
     char line[1024];
     int line_num = 0;
     bool success = true;

     fp = fopen(filepath, "r");
     if (!fp) {
         fprintf(stderr, "ERROR: Cannot open mapping file '%s'.\n", filepath);
         return false;
     }

     while (fgets(line, sizeof(line), fp)) {
         char *trimmed_line = line;
         char *tab1, *tab2, *tab3;
         char *endptr_seg, *endptr_msg;
         long seg_idx_long, msg_idx_long;
         MessageMapping entry;
         char *filename_start, *comment_start, *end;

         line_num++;

         /* Trim leading whitespace */
         while (isspace((unsigned char)*trimmed_line))
             trimmed_line++;

         /* Skip empty lines and comments starting at beginning of line */
         if (trimmed_line[0] == '\0' || trimmed_line[0] == '#')
             continue;

         /* Find tabs */
         tab1 = strchr(trimmed_line, '\t');
         tab2 = tab1 ? strchr(tab1 + 1, '\t') : NULL;
         tab3 = tab2 ? strchr(tab2 + 1, '\t') : NULL; /* Optional third tab */

         if (!tab1 || !tab2) {
             fprintf(stderr, "ERROR: Invalid format in mapping file '%s' at line %d: Missing tabs.\n", filepath, line_num);
             success = false;
             break;
         }

         *tab1 = '\0'; /* Null-terminate segment index */
         *tab2 = '\0'; /* Null-terminate message index */

         seg_idx_long = strtol(trimmed_line, &endptr_seg, 10);
         msg_idx_long = strtol(tab1 + 1, &endptr_msg, 10);

         /* Validate indices (Segment index must be >= 0, Message index >= 0) */
         if (*endptr_seg != '\0' || *endptr_msg != '\0' || seg_idx_long < 0 || msg_idx_long < 0) {
             fprintf(stderr, "ERROR: Invalid index format in mapping file '%s' at line %d (ensure SegIdx >= 0, MsgIdxInSeg >= 0).\n", filepath, line_num);
             success = false;
             break;
         }

         entry.segment_index = (int)seg_idx_long; /* Store 0-based index */
         entry.message_index_in_seg = (int)msg_idx_long;

         filename_start = tab2 + 1;
         comment_start = NULL;

         /* Handle filename and potential comment */
         if (tab3) {
             /* Comment exists */
             char *filename_end = tab3 - 1;
             /* Trim trailing whitespace from filename */
             while (filename_end >= filename_start && isspace((unsigned char)*filename_end)) {
                 *filename_end-- = '\0';
             }
             *tab3 = '\0'; /* Null-terminate filename cleanly */
             comment_start = tab3 + 1;
         } else {
             /* No comment field, trim trailing whitespace from filename */
             end = filename_start + strlen(filename_start) - 1;
             while (end >= filename_start && isspace((unsigned char)*end)) {
                 *end-- = '\0';
             }
         }

         entry.output_filename_base = strdup(filename_start);
         if (!entry.output_filename_base) {
             fprintf(stderr, "ERROR: Memory allocation failed for filename (line %d).\n", line_num);
             success = false;
             break;
         }

         if (comment_start && *comment_start != '\0') {
             /* Trim trailing newline/whitespace from comment */
             end = comment_start + strlen(comment_start) - 1;
             while (end >= comment_start && isspace((unsigned char)*end))
                 *end-- = '\0';

             entry.comment = strdup(comment_start);
             if (!entry.comment) {
                 fprintf(stderr, "ERROR: Memory allocation failed for comment (line %d).\n", line_num);
                 free(entry.output_filename_base); /* Clean up */
                 success = false;
                 break;
             }
             /* Clean comment AFTER saving it, before adding to table */
             clean_comment(entry.comment);
         } else {
             entry.comment = NULL; /* No comment provided */
         }

         if (!add_mapping(table, entry)) {
             /* Error printed in add_mapping, entry strings already freed */
             success = false;
             break;
         }
     }

     fclose(fp);
     if (!success)
         free_mapping_table(table); /* Clean up partially loaded table */

     return success;
 }

 /**
  * find_mapping() - Finds a mapping entry in the table using 0-based indices.
  * @table:                 Pointer to the MappingTable.
  * @segment_index_0_based: 0-based segment index.
  * @message_index_in_seg:  0-based message index within the segment.
  *
  * Return: Pointer to the found MessageMapping, or NULL if not found.
  */
 const MessageMapping *
 find_mapping(const MappingTable *table, int segment_index_0_based, int message_index_in_seg)
 {
     size_t i;

     if (!table || !table->mappings)
         return NULL;

     for (i = 0; i < table->count; ++i) {
         if (table->mappings[i].segment_index == segment_index_0_based &&
             table->mappings[i].message_index_in_seg == message_index_in_seg)
             return &table->mappings[i];
     }
     return NULL;
 }


 /* --- PCM Buffer Handling --- */

 /**
  * init_pcm_buffer() - Initializes a PcmBuffer.
  * @buffer: Pointer to the PcmBuffer.
  */
 void
 init_pcm_buffer(PcmBuffer *buffer)
 {
     buffer->samples = NULL;
     buffer->count = 0;
     buffer->capacity = 0;
 }

 /**
  * add_pcm_sample() - Adds a PCM sample to the buffer, resizing if necessary.
  * @buffer: Pointer to the PcmBuffer.
  * @sample: The 16-bit PCM sample to add.
  *
  * Return: true on success, false on memory allocation failure.
  */
 bool
 add_pcm_sample(PcmBuffer *buffer, int16_t sample)
 {
     if (buffer->count >= buffer->capacity) {
         size_t new_capacity = (buffer->capacity == 0) ? 2048 : buffer->capacity * 2;
         int16_t *new_samples;

         /* Prevent excessively large allocations */
         if (new_capacity > SIZE_MAX / sizeof(int16_t) / 2) {
             fprintf(stderr, "ERROR: PCM buffer capacity exceeds limit.\n");
             return false;
         }
         new_samples = (int16_t *)realloc(buffer->samples, new_capacity * sizeof(int16_t));
         if (!new_samples) {
             fprintf(stderr, "ERROR: Failed to reallocate memory for PCM buffer (capacity %zu).\n", new_capacity);
             return false;
         }
         buffer->samples = new_samples;
         buffer->capacity = new_capacity;
     }
     buffer->samples[buffer->count++] = sample;
     return true;
 }

 /**
  * free_pcm_buffer() - Frees memory associated with a PcmBuffer.
  * @buffer: Pointer to the PcmBuffer.
  */
 void
 free_pcm_buffer(PcmBuffer *buffer)
 {
     free(buffer->samples);
     buffer->samples = NULL;
     buffer->count = 0;
     buffer->capacity = 0;
 }

 /* --- ADPCM Decoding --- */

 /**
  * decode_nibble() - Decodes a single 4-bit ADPCM nibble using updated tables.
  * @nibble:     The 4-bit ADPCM nibble (0-15).
  * @state:      Pointer to the current AdpcmState.
  * @pcm_buffer: Pointer to the PcmBuffer to add the sample to.
  *
  * Return: true on success, false on failure (e.g., buffer allocation).
  */
 bool
 decode_nibble(uint8_t nibble, AdpcmState *state, PcmBuffer *pcm_buffer)
 {
     int diff; /* Signed difference from table */
     int16_t pcm_sample;
     int next_state;

     /* Ensure state index is valid */
     if (state->adpcm_state < 0 || state->adpcm_state > 15) {
         fprintf(stderr, "ERROR: Invalid ADPCM state index %d\n", state->adpcm_state);
         state->adpcm_state = 0; /* Reset state */
     }

     /* Get difference from 2D step table */
     diff = step_table[state->adpcm_state][nibble];

     /* Update sample value */
     /* Note: Using int32_t for intermediate calculation to avoid overflow before clamping */
     int32_t next_sample = (int32_t)state->current_sample + diff;

     /* Clamp sample (important for ADPCM) */
     if (next_sample > 32767)
         next_sample = 32767;
     else if (next_sample < -32768)
         next_sample = -32768;
     state->current_sample = (int16_t)next_sample;

     /* Update state index using state table */
     next_state = state->adpcm_state + state_table[nibble];

     /* Clamp state index */
     if (next_state < 0)
         next_state = 0;
     else if (next_state > 15)
         next_state = 15;
     state->adpcm_state = (int8_t)next_state;

     /* Scale to 16-bit PCM (as per original specification's hint) and add to buffer */
     /* Clamping might make this scaling less critical, but retain for consistency */
     pcm_sample = (int16_t)(state->current_sample << 7);
     /* Re-clamp after scaling, just in case << 7 causes overflow */
     if ((state->current_sample > (32767 >> 7)) && (diff > 0)) pcm_sample = 32767;
     if ((state->current_sample < (-32768 >> 7)) && (diff < 0)) pcm_sample = -32768;

     return add_pcm_sample(pcm_buffer, pcm_sample);
 }


 /* --- WAV File Writing --- */

 /**
  * write_info_sub_chunk() - Writes a metadata sub-chunk to the WAV file.
  * @id:   The 4-character chunk ID.
  * @text: The string data for the chunk.
  * @fp:   File pointer.
  *
  * Return: The number of bytes written (including ID, size, data, padding),
  * or 0 on error.
  */
 uint32_t
 write_info_sub_chunk(const char *id, const char *text, FILE *fp)
 {
     size_t text_len;
     uint32_t chunk_size;
     bool needs_padding;
     uint32_t total_size;

     if (!text)
         return 0; /* Skip if text is NULL */

     text_len = strlen(text);
     chunk_size = (uint32_t)text_len + 1; /* Include null terminator */
     needs_padding = (chunk_size % 2 != 0);
     total_size = 4 + 4 + chunk_size + (needs_padding ? 1 : 0);

     if (!write_chunk_id(id, fp)) return 0;
     if (!write_u32le(chunk_size, fp)) return 0;
     if (fwrite(text, 1, text_len + 1, fp) != text_len + 1) return 0; /* Write string + null */
     if (needs_padding) {
         uint8_t padding_byte = 0;
         if (fwrite(&padding_byte, 1, 1, fp) != 1) return 0;
     }

     return total_size;
 }

 /**
  * write_wav_file() - Writes decoded PCM data to a WAV file with metadata.
  * @output_filepath:    Full path for the output WAV file.
  * @pcm_buffer:         Pointer to the PcmBuffer containing the samples.
  * @sample_rate:        Sample rate (e.g., 8000).
  * @rom_basename:       Base filename of the input ROM (for Artist tag).
  * @track_title:        Title for the track (INAM tag).
  * @track_number_str:   String representation of the absolute track number.
  * @comment:            Comment string (ICMT tag, can be NULL).
  *
  * Return: true on success, false on failure.
  */
 bool
 write_wav_file(const char *output_filepath, const PcmBuffer *pcm_buffer,
            uint32_t sample_rate, const char *rom_basename,
            const char *track_title, const char *track_number_str,
            const char *comment)
 {
     FILE *fp;
     bool success = false; /* Assume failure */
     char date_str[11]; /* YYYY-MM-DD */
     time_t now;
     struct tm *t;
     const char *album = "Nortel Millennium VoiceWare";
     const char *artist = rom_basename;
     uint32_t num_samples, bytes_per_sample, data_chunk_size;
     uint64_t data_chunk_size_64;
     bool data_needs_padding;
     uint32_t padded_data_chunk_size;
     uint32_t info_chunk_total_size, info_chunk_data_size;
     size_t temp_len;
     bool temp_pad;
     uint32_t fmt_chunk_size, riff_chunk_size, bytes_per_sec;
     uint16_t block_align;
     size_t i;

     fp = fopen(output_filepath, "wb");
     if (!fp) {
         fprintf(stderr, "ERROR: Cannot open output WAV file '%s' for writing.\n", output_filepath);
         return false;
     }

     /* --- Prepare Metadata --- */
     now = time(NULL);
     t = localtime(&now);
     strftime(date_str, sizeof(date_str), "%Y-%m-%d", t);

     /* --- Calculate Sizes --- */
     num_samples = (uint32_t)pcm_buffer->count;
     bytes_per_sample = ADPCM_BITS / 8;
     data_chunk_size_64 = (uint64_t)num_samples * bytes_per_sample;

     /* Check for data chunk size overflow */
     if (data_chunk_size_64 > UINT32_MAX) {
         fprintf(stderr, "ERROR: WAV data chunk size exceeds 4GB limit for message '%s'.\n", track_title);
         goto cleanup;
     }
     data_chunk_size = (uint32_t)data_chunk_size_64;
     data_needs_padding = (data_chunk_size % 2 != 0);
     padded_data_chunk_size = data_chunk_size + (data_needs_padding ? 1 : 0);


     /* Calculate LIST/INFO chunk size */
     info_chunk_total_size = 4; /* LIST chunk ID "LIST" */
     info_chunk_data_size = 4; /* Type ID "INFO" */

     /* IALB */
     temp_len = strlen(album) + 1; temp_pad = temp_len % 2 != 0;
     info_chunk_data_size += 4 + 4 + temp_len + (temp_pad ? 1 : 0);
     /* IART */
     temp_len = strlen(artist) + 1; temp_pad = temp_len % 2 != 0;
     info_chunk_data_size += 4 + 4 + temp_len + (temp_pad ? 1 : 0);
     /* INAM */
     temp_len = strlen(track_title) + 1; temp_pad = temp_len % 2 != 0;
     info_chunk_data_size += 4 + 4 + temp_len + (temp_pad ? 1 : 0);
     /* ITRK */
     temp_len = strlen(track_number_str) + 1; temp_pad = temp_len % 2 != 0;
     info_chunk_data_size += 4 + 4 + temp_len + (temp_pad ? 1 : 0);
     /* ICRD */
     temp_len = strlen(date_str) + 1; temp_pad = temp_len % 2 != 0;
     info_chunk_data_size += 4 + 4 + temp_len + (temp_pad ? 1 : 0);
     /* ICMT */
     if (comment && strlen(comment) > 0) {
          temp_len = strlen(comment) + 1; temp_pad = temp_len % 2 != 0;
          info_chunk_data_size += 4 + 4 + temp_len + (temp_pad ? 1 : 0);
     }

     info_chunk_total_size += info_chunk_data_size; /* Add size field itself */


     /* RIFF Chunk Size */
     fmt_chunk_size = 16; /* For standard PCM */
     riff_chunk_size = 4 + /* "WAVE" ID */
               (4 + 4 + fmt_chunk_size) + /* "fmt " chunk */
               info_chunk_total_size +     /* "LIST" chunk */
               (4 + 4 + padded_data_chunk_size); /* "data" chunk */

     /* --- Write RIFF Header --- */
     if (!write_chunk_id("RIFF", fp)) goto cleanup;
     if (!write_u32le(riff_chunk_size, fp)) goto cleanup;
     if (!write_chunk_id("WAVE", fp)) goto cleanup;

     /* --- Write "fmt " Chunk --- */
     if (!write_chunk_id("fmt ", fp)) goto cleanup;
     if (!write_u32le(fmt_chunk_size, fp)) goto cleanup; /* Size of chunk data */
     if (!write_u16le(1, fp)) goto cleanup;             /* wFormatTag (1 = PCM) */
     if (!write_u16le(ADPCM_CHANNELS, fp)) goto cleanup; /* nChannels */
     if (!write_u32le(sample_rate, fp)) goto cleanup;    /* nSamplesPerSec */
     bytes_per_sec = sample_rate * ADPCM_CHANNELS * bytes_per_sample;
     if (!write_u32le(bytes_per_sec, fp)) goto cleanup; /* nAvgBytesPerSec */
     block_align = ADPCM_CHANNELS * bytes_per_sample;
     if (!write_u16le(block_align, fp)) goto cleanup;   /* nBlockAlign */
     if (!write_u16le(ADPCM_BITS, fp)) goto cleanup;    /* wBitsPerSample */

     /* --- Write "LIST" (INFO) Chunk --- */
     if (!write_chunk_id("LIST", fp)) goto cleanup;
     if (!write_u32le(info_chunk_data_size, fp)) goto cleanup; /* Size of LIST data */
     if (!write_chunk_id("INFO", fp)) goto cleanup; /* List type */

     if (write_info_sub_chunk("IALB", album, fp) == 0) goto cleanup;
     if (write_info_sub_chunk("IART", artist, fp) == 0) goto cleanup;
     if (write_info_sub_chunk("INAM", track_title, fp) == 0) goto cleanup;
     if (write_info_sub_chunk("ITRK", track_number_str, fp) == 0) goto cleanup;
     if (write_info_sub_chunk("ICRD", date_str, fp) == 0) goto cleanup;
     if (comment && strlen(comment) > 0) {
         if (write_info_sub_chunk("ICMT", comment, fp) == 0) goto cleanup;
     }

     /* --- Write "data" Chunk --- */
     if (!write_chunk_id("data", fp)) goto cleanup;
     if (!write_u32le(data_chunk_size, fp)) goto cleanup; /* Actual data size */

     /* Write sample data explicitly as Little Endian */
     for (i = 0; i < pcm_buffer->count; ++i) {
         if (!write_u16le((uint16_t)pcm_buffer->samples[i], fp)) goto cleanup;
     }

     /* Add padding byte if data chunk size was odd */
     if (data_needs_padding) {
         uint8_t padding_byte = 0;
         if (fwrite(&padding_byte, 1, 1, fp) != 1) goto cleanup;
     }

     /* If we reached here, writing was successful */
     success = true;
     status_printf("Successfully wrote WAV: %s (%u samples)\n", output_filepath, num_samples);


 cleanup:
     if (!success)
         fprintf(stderr, "ERROR: Failed to write WAV file '%s'.\n", output_filepath);

     fclose(fp);
     /* If writing failed, consider deleting the incomplete file */
     /* if (!success) remove(output_filepath); */
     return success;
 }

 /* --- Raw PCM Saving --- */

 /**
  * save_raw_pcm() - Saves raw message data to a .pcm file.
  * @output_filepath:      Full path for the output .pcm file.
  * @rom_data:             Pointer to the start of the ROM data buffer.
  * @message_start_offset: Offset of the message's mode byte.
  * @message_end_offset:   Offset of the byte *after* the last byte of message.
  *
  * Return: true on success, false on failure.
  */
 bool
 save_raw_pcm(const char *output_filepath, const uint8_t *rom_data,
          size_t message_start_offset, size_t message_end_offset)
 {
     FILE *fp;
     size_t data_size, written;

     if (message_end_offset <= message_start_offset) {
          fprintf(stderr, "ERROR: Invalid offsets for saving raw PCM for '%s'.\n", output_filepath);
          return false;
     }

     fp = fopen(output_filepath, "wb");
     if (!fp) {
         fprintf(stderr, "ERROR: Cannot open output PCM file '%s' for writing.\n", output_filepath);
         return false;
     }

     data_size = message_end_offset - message_start_offset;
     written = fwrite(rom_data + message_start_offset, 1, data_size, fp);

     fclose(fp);

     if (written != data_size) {
         fprintf(stderr, "ERROR: Failed to write complete data to PCM file '%s'.\n", output_filepath);
         /* remove(output_filepath); */
         return false;
     }

     status_printf("Saved raw PCM data: %s (%zu bytes)\n", output_filepath, data_size);

     return true;
 }


 /* --- Message Processing --- */

 /**
  * process_message() - Processes a single message (ADPCM decoding or Raw PCM saving).
  * NOTE: This function is NOT called when list_mode is active.
  * @rom_data:             Pointer to the start of the ROM data buffer.
  * @rom_size:             Total size of the ROM data.
  * @segment_start_offset: Byte offset of the current segment's start.
  * @segment_index_0_based: 0-based index of the current segment.
  * @msg_idx_in_segment:   0-based index of the message within the segment.
  * @absolute_msg_idx:     0-based absolute index of the message.
  * @message_offset_in_segment: Offset (bytes) from segment start to mode byte.
  * @next_message_offset_in_segment: Offset (bytes) of the *next* message.
  * @mapping:              Pointer to mapping info (or NULL if none).
  * @rom_basename:         Base filename of the input ROM file.
  *
  * Return: true if processing should continue, false on fatal error.
  */
 bool
 process_message(const uint8_t *rom_data, size_t rom_size,
         size_t segment_start_offset, int segment_index_0_based,
         int msg_idx_in_segment, int absolute_msg_idx,
         uint32_t message_offset_in_segment, uint32_t next_message_offset_in_segment,
         const MessageMapping *mapping, const char *rom_basename)
 {
     size_t start_address = segment_start_offset + message_offset_in_segment;
     uint8_t message_mode;
     size_t current_pos;
     char default_filename_base[25]; /* "message_S_XXX" + buffer */
     const char *output_base;
     const char *comment = NULL;

     /* Basic bounds check for start address */
     if (start_address >= rom_size) {
         fprintf(stderr, "WARN: Calculated start address (0x%zX) for message %d (Seg %d, Idx %d) is out of bounds (ROM size 0x%zX). Skipping.\n",
             start_address, absolute_msg_idx, segment_index_0_based, msg_idx_in_segment, rom_size);
         return true; /* Continue processing other messages */
     }

     message_mode = rom_data[start_address];
     current_pos = start_address + 1; /* Position for reading commands/data */

     /* Generate default filename: message_S_XXX (0-based indices) */
     snprintf(default_filename_base, sizeof(default_filename_base), "message_%d_%03d",
          segment_index_0_based, msg_idx_in_segment);

     output_base = default_filename_base;
     if (mapping) {
         output_base = mapping->output_filename_base;
         comment = mapping->comment;
     }

     status_printf("Processing Message: Absolute Index %d (Segment %d, Index %d), Mode 0x%02X, Offset 0x%zX\n",
            absolute_msg_idx, segment_index_0_based, msg_idx_in_segment, message_mode, start_address);

     if (message_mode == MODE_ADPCM) {
         AdpcmState adpcm_state = {0, 0}; /* Initial state */
         PcmBuffer pcm_buffer;
         bool decoding_ok = true;
         bool end_of_message = false;
         uint32_t nibble_count = 0;
         uint8_t repeat_count = 0; /* Times to repeat *next* block (0=play once) */
         uint32_t current_repeat_nibble_start = 0;
         uint32_t current_repeat_nibble_count = 0;

         verbose_printf("  Type: ADPCM\n");
         init_pcm_buffer(&pcm_buffer);

         while (!end_of_message && current_pos < rom_size) {
             /* --- Nibble Decoding Phase --- */
             if (nibble_count > 0) {
                 uint8_t data_byte, nibble1, nibble2;

                 if (current_pos >= rom_size) {
                     fprintf(stderr, "WARN: Unexpected end of ROM data while reading ADPCM data nibble for message %d.\n", absolute_msg_idx);
                     decoding_ok = false;
                     break;
                 }
                 data_byte = rom_data[current_pos++];
                 nibble1 = (data_byte >> 4) & 0x0F; /* MSN */
                 nibble2 = data_byte & 0x0F;        /* LSN */

                 verbose_printf("    Nibble Read: Byte 0x%02X -> N1=0x%X, N2=0x%X (Pos 0x%zX)\n", data_byte, nibble1, nibble2, current_pos - 1);

                 /* Decode first nibble */
                 if (!decode_nibble(nibble1, &adpcm_state, &pcm_buffer)) {
                     decoding_ok = false; break;
                 }
                 nibble_count--;

                 /* Decode second nibble if needed */
                 if (nibble_count > 0) {
                     if (!decode_nibble(nibble2, &adpcm_state, &pcm_buffer)) {
                         decoding_ok = false; break;
                     }
                     nibble_count--;
                 }

                 /* Handle repeat logic */
                 if (repeat_count > 0 && nibble_count == 0) {
                     repeat_count--;
                     if (repeat_count > 0) {
                         /* Reset position and count to repeat block */
                         verbose_printf("    Repeating block (%u nibbles left, %u repeats left)\n", current_repeat_nibble_count, repeat_count);
                         current_pos = current_repeat_nibble_start;
                         nibble_count = current_repeat_nibble_count;
                     } else {
                          verbose_printf("    Finished repeating block.\n");
                          current_repeat_nibble_start = 0;
                          current_repeat_nibble_count = 0;
                     }
                 }
             }
             /* --- Command Reading Phase --- */
             else {
                 uint8_t command;

                 if (current_pos >= rom_size) {
                     fprintf(stderr, "WARN: Unexpected end of ROM data while reading ADPCM command for message %d.\n", absolute_msg_idx);
                     end_of_message = (pcm_buffer.count > 0);
                     decoding_ok = end_of_message;
                     break;
                 }
                 command = rom_data[current_pos++];
                 verbose_printf("  Command Read: 0x%02X (Pos 0x%zX)\n", command, current_pos - 1);

                 if (command == 0x00) { /* End of Message */
                     verbose_printf("    Opcode: End of Message\n");
                     end_of_message = true;
                 } else if (command >= 0x01 && command <= 0x3F) { /* Silence */
                     uint32_t silence_samples = (uint32_t)command * 8;
                     uint32_t i;
                     verbose_printf("    Opcode: Silence (%u samples)\n", silence_samples);
                     for (i = 0; i < silence_samples; ++i) {
                         if (!add_pcm_sample(&pcm_buffer, 0)) {
                             decoding_ok = false; break;
                         }
                     }
                 } else if (command >= 0x40 && command <= 0x7F) { /* Play Short Block */
                     nibble_count = 256; /* 128 bytes * 2 nibbles/byte */
                     repeat_count = 0;
                     verbose_printf("    Opcode: Play Short Block (%u nibbles)\n", nibble_count);
                 } else if (command >= 0x80 && command <= 0xBF) { /* Play Long Block */
                     uint8_t n;
                     if (current_pos >= rom_size) {
                         fprintf(stderr, "WARN: Unexpected end of ROM reading N for Long Block (Cmd 0x%02X) in message %d.\n", command, absolute_msg_idx);
                         decoding_ok = false; break;
                     }
                     n = rom_data[current_pos++];
                     nibble_count = (uint32_t)n + 1;
                     repeat_count = 0;
                     verbose_printf("    Opcode: Play Long Block (N=0x%02X -> %u nibbles) (Pos 0x%zX)\n", n, nibble_count, current_pos - 1);
                 } else if (command >= 0xC0 && command <= 0xFF) { /* Play Repeat Block */
                     uint8_t n;
                     if (current_pos >= rom_size) {
                         fprintf(stderr, "WARN: Unexpected end of ROM reading N for Repeat Block (Cmd 0x%02X) in message %d.\n", command, absolute_msg_idx);
                         decoding_ok = false; break;
                     }
                     n = rom_data[current_pos++];
                     nibble_count = (uint32_t)n + 1;
                     repeat_count = ((command >> 3) & 0x07); /* R bits (0-7) */
                     current_repeat_nibble_start = current_pos;
                     current_repeat_nibble_count = nibble_count;
                     verbose_printf("    Opcode: Play Repeat Block (N=0x%02X -> %u nibbles, R=%u -> %u plays total) (Pos 0x%zX)\n",
                                n, nibble_count, repeat_count, repeat_count + 1, current_pos - 1);
                 } else {
                     fprintf(stderr, "WARN: Unknown ADPCM command byte 0x%02X at offset 0x%zX in message %d. Stopping decode.\n",
                         command, current_pos - 1, absolute_msg_idx);
                     decoding_ok = false; /* Treat as error */
                     break;
                 }
                 if (!decoding_ok) break; /* Break outer loop on error */
             }
         } /* end while(!end_of_message) */

         if (decoding_ok && pcm_buffer.count > 0) {
             char wav_filename[FILENAME_MAX];
             char track_num_str[12];

             snprintf(wav_filename, sizeof(wav_filename), "%s.wav", output_base);
             snprintf(track_num_str, sizeof(track_num_str), "%d", absolute_msg_idx);

             if (!write_wav_file(wav_filename, &pcm_buffer, DEFAULT_SAMPLE_RATE,
                         rom_basename, output_base, track_num_str, comment)) {
                 /* Error already printed */
             }
         } else if (decoding_ok && pcm_buffer.count == 0) {
             status_printf("  Message %d resulted in 0 PCM samples. No WAV file written.\n", absolute_msg_idx);
         } else {
              fprintf(stderr, "ERROR: Decoding failed for message %d. No WAV file written.\n", absolute_msg_idx);
         }

         free_pcm_buffer(&pcm_buffer);

     } else if (message_mode == MODE_PCM) {
         size_t message_end_offset;
         char pcm_filename[FILENAME_MAX];

         verbose_printf("  Type: Raw PCM (Saving raw data, decoding not supported)\n");
         /* Determine end of message data */
         message_end_offset = segment_start_offset + next_message_offset_in_segment;
         if (message_end_offset > rom_size) /* Clamp to ROM size */
             message_end_offset = rom_size;

         snprintf(pcm_filename, sizeof(pcm_filename), "%s.pcm", output_base);

         if (message_end_offset <= start_address) {
              fprintf(stderr, "WARN: Cannot determine valid data range for Raw PCM message %d. Skipping save.\n", absolute_msg_idx);
         } else {
             if (!save_raw_pcm(pcm_filename, rom_data, start_address, message_end_offset)) {
                 /* Error already printed */
             }
         }

     } else {
         fprintf(stderr, "WARN: Unknown message mode 0x%02X for message %d at offset 0x%zX. Skipping.\n",
             message_mode, absolute_msg_idx, start_address);
     }

     return true; /* Continue processing next message */
 }

 /**
  * handle_message_iteration() - Handles a single message during iteration (list or decode).
  * @rom_data:             Pointer to the start of the ROM data buffer.
  * @rom_size:             Total size of the ROM data.
  * @segment_start_offset: Byte offset of the current segment's start.
  * @segment_index_0_based: 0-based index of the current segment.
  * @msg_idx_in_seg:       0-based index of the message within the segment.
  * @absolute_msg_idx:     0-based absolute index of the message.
  * @offset_table:         Pointer to the offset table for the current segment.
  * @message_count_in_segment: Total number of messages in the current segment.
  * @mapping_table:        Pointer to the loaded mapping table.
  * @rom_basename:         Base filename of the input ROM file.
  * @list_mode:            True if list mode is active.
  * @quiet_mode:           True if quiet mode is active.
  * @target_message_idx:   Target absolute message index for decoding (-1 for all).
  *
  * Return: Enum indicating status (continue, target found, error).
  */
 HandleMessageResult
 handle_message_iteration(
     const uint8_t *rom_data, size_t rom_size,
     size_t segment_start_offset, int segment_index_0_based,
     uint32_t msg_idx_in_seg, int absolute_msg_idx,
     const uint16_t *offset_table, uint32_t message_count_in_segment,
     const MappingTable *mapping_table, const char *rom_basename,
     bool list_mode, bool quiet_mode, long target_message_idx)
 {
     const MessageMapping *mapping = find_mapping(mapping_table, segment_index_0_based, msg_idx_in_seg);
     uint32_t relative_base_offset_words = offset_table[msg_idx_in_seg];
     uint32_t message_offset_bytes = (uint32_t)relative_base_offset_words * 2;
     size_t start_address = segment_start_offset + message_offset_bytes;

     /* --- LIST MODE --- */
     if (list_mode) {
         /* Only proceed with list output if not in quiet mode */
         if (!quiet_mode) {
             const char *output_base;
             const char *user_comment = NULL;
             char default_filename_base[25];
             uint8_t message_mode = 0xFF;
             bool mode_read_ok = false;
             char comment_buffer[256] = "";
             bool pcm_tag_added = false;
             bool pcm_already_in_user_comment = false;
             bool has_user_comment = false;
             int filename_len, num_stops, target_stops, tabs_to_print, pad_idx;

             /* Determine filename base and check existing comment for (PCM) */
             if (mapping) {
                 output_base = mapping->output_filename_base;
                 user_comment = mapping->comment;
                 if (user_comment) {
                     pcm_already_in_user_comment = (strstr(user_comment, "(PCM)") != NULL);
                     has_user_comment = (strlen(user_comment) > 0);
                 }
             } else {
                 snprintf(default_filename_base, sizeof(default_filename_base), "message_%d_%03d",
                      segment_index_0_based, msg_idx_in_seg);
                 output_base = default_filename_base;
             }

             /* Read message mode for PCM check */
             if (start_address < rom_size) {
                 message_mode = rom_data[start_address];
                 mode_read_ok = true;
             } else {
                 fprintf(stderr, "WARN: Cannot read mode byte for list entry (Seg %d, Idx %u) - offset out of bounds.\n",
                     segment_index_0_based, msg_idx_in_seg);
             }

             /* Build the comment string */
             strcpy(comment_buffer, "#"); /* Start with hash */
             if (mode_read_ok && message_mode == MODE_PCM && !pcm_already_in_user_comment) {
                 strcat(comment_buffer, " (PCM)");
                 pcm_tag_added = true;
             }
             if (has_user_comment) {
                 if (pcm_tag_added || strcmp(comment_buffer, "#") == 0)
                      strcat(comment_buffer, " ");
                 strcat(comment_buffer, user_comment);
             } else if (!pcm_tag_added) {
                 strcat(comment_buffer, " ");
             }


             /* Print first fields */
             printf("%d\t%u\t%s", segment_index_0_based, msg_idx_in_seg, output_base);

             /* Calculate and print padding TABS */
             filename_len = strlen(output_base);
             num_stops = filename_len / TAB_WIDTH;
             target_stops = (LIST_FILENAME_ALIGN_WIDTH + TAB_WIDTH - 1) / TAB_WIDTH;
             tabs_to_print = (num_stops < target_stops) ? (target_stops - num_stops) : 1;
             for (pad_idx = 0; pad_idx < tabs_to_print; ++pad_idx)
                 putchar('\t');

             /* Print comment */
             printf("%s\n", comment_buffer);
         }
         return MSG_HANDLED_CONTINUE; /* List mode always continues */
     }
     /* --- DECODE MODE --- */
     else {
         if (target_message_idx < 0 || absolute_msg_idx == target_message_idx) {
             uint32_t next_message_offset_bytes;
             bool success;

             /* Determine the end offset for Raw PCM saving */
             if (msg_idx_in_seg + 1 < message_count_in_segment) {
                 next_message_offset_bytes = (uint32_t)offset_table[msg_idx_in_seg + 1] * 2;
             } else {
                 next_message_offset_bytes = ROM_SEGMENT_SIZE; /* Assume end of segment */
             }

             success = process_message(rom_data, rom_size, segment_start_offset, segment_index_0_based,
                           msg_idx_in_seg, absolute_msg_idx,
                           message_offset_bytes, next_message_offset_bytes,
                           mapping, rom_basename);

             if (!success)
                 return MSG_HANDLED_ERROR;

             if (target_message_idx >= 0 && absolute_msg_idx == target_message_idx)
                 return MSG_HANDLED_TARGET_FOUND;

         }
         return MSG_HANDLED_CONTINUE; /* Continue if not target or target processed ok */
     }
 }


 /* --- Argument Parsing Function --- */

 /**
  * parse_arguments() - Parses command line arguments.
  * @argc: Argument count.
  * @argv: Argument vector.
  * @rom_filepath_ptr: Pointer to store ROM file path.
  * @map_filepath_ptr: Pointer to store mapping file path.
  * @target_message_idx_ptr: Pointer to store target message index.
  * @list_mode_ptr: Pointer to store list mode flag.
  * @quiet_mode_ptr: Pointer to store quiet mode flag.
  * @verbose_mode_ptr: Pointer to store verbose mode flag.
  *
  * Return: true on success, false on error or if help requested.
  */
 bool
 parse_arguments(int argc, char *argv[],
         const char **rom_filepath_ptr, const char **map_filepath_ptr,
         long *target_message_idx_ptr, bool *list_mode_ptr,
         bool *quiet_mode_ptr, bool *verbose_mode_ptr)
 {
     int i;

     /* Initialize defaults */
     *rom_filepath_ptr = NULL;
     *map_filepath_ptr = NULL;
     *target_message_idx_ptr = -1;
     *list_mode_ptr = false;
     *quiet_mode_ptr = false;
     *verbose_mode_ptr = false;

     for (i = 1; i < argc; ++i) {
         if (strcmp(argv[i], "-m") == 0) {
             if (++i < argc) {
                 *map_filepath_ptr = argv[i];
             } else {
                 fprintf(stderr, "ERROR: Option -m requires a filepath argument.\n");
                 print_usage(argv[0]);
                 return false;
             }
         } else if (strcmp(argv[i], "-i") == 0) {
             if (++i < argc) {
                 char *endptr;
                 *target_message_idx_ptr = strtol(argv[i], &endptr, 10);
                 if (*endptr != '\0' || *target_message_idx_ptr < 0) {
                     fprintf(stderr, "ERROR: Invalid message index '%s' for -i option.\n", argv[i]);
                     print_usage(argv[0]);
                     return false;
                 }
             } else {
                 fprintf(stderr, "ERROR: Option -i requires a message index argument.\n");
                 print_usage(argv[0]);
                 return false;
             }
         } else if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--list") == 0) {
             *list_mode_ptr = true;
         } else if (strcmp(argv[i], "-q") == 0 || strcmp(argv[i], "--quiet") == 0) {
             *quiet_mode_ptr = true;
         } else if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
             *verbose_mode_ptr = true;
         } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
              print_usage(argv[0]);
              return false; /* Indicate help requested, not an error */
         } else if (argv[i][0] == '-') {
             fprintf(stderr, "ERROR: Unknown option '%s'.\n", argv[i]);
             print_usage(argv[0]);
             return false;
         } else if (*rom_filepath_ptr == NULL) {
             *rom_filepath_ptr = argv[i];
         } else {
             fprintf(stderr, "ERROR: Unexpected argument '%s'. ROM filepath already specified?\n", argv[i]);
             print_usage(argv[0]);
             return false;
         }
     }

     if (*rom_filepath_ptr == NULL) {
         fprintf(stderr, "ERROR: Input ROM filepath is required.\n");
         print_usage(argv[0]);
         return false;
     }

     /* Quiet mode overrides verbose mode */
     if (*quiet_mode_ptr)
         *verbose_mode_ptr = false;


     /* If listing, ignore target index */
     if (*list_mode_ptr && *target_message_idx_ptr >= 0) {
         status_printf("INFO: Option -i ignored when -l or --list is specified.\n");
         *target_message_idx_ptr = -1; /* Ensure we don't accidentally use it later */
     }

     return true;
 }

 /**
  * load_rom_data() - Loads ROM file content into memory.
  * @rom_filepath: Path to the ROM file.
  * @rom_data_ptr: Pointer to store the allocated buffer address.
  * @rom_size_ptr: Pointer to store the size of the loaded ROM.
  *
  * Return: true on success, false on failure.
  */
 bool
 load_rom_data(const char *rom_filepath, uint8_t **rom_data_ptr, size_t *rom_size_ptr)
 {
     FILE *rom_fp;
     long file_size_long;

     verbose_printf("Loading ROM file...\n");
     rom_fp = fopen(rom_filepath, "rb");
     if (!rom_fp) {
         fprintf(stderr, "ERROR: Cannot open ROM file '%s'.\n", rom_filepath);
         return false;
     }

     fseek(rom_fp, 0, SEEK_END);
     file_size_long = ftell(rom_fp);
     fseek(rom_fp, 0, SEEK_SET);

     if (file_size_long < 0 || file_size_long == 0) {
          fprintf(stderr, "ERROR: Invalid ROM file size (%ld).\n", file_size_long);
          fclose(rom_fp);
          return false;
     }
     *rom_size_ptr = (size_t)file_size_long;


     *rom_data_ptr = (uint8_t *)malloc(*rom_size_ptr);
     if (!*rom_data_ptr) {
         fprintf(stderr, "ERROR: Failed to allocate %zu bytes for ROM data.\n", *rom_size_ptr);
         fclose(rom_fp);
         return false;
     }

     if (fread(*rom_data_ptr, 1, *rom_size_ptr, rom_fp) != *rom_size_ptr) {
         fprintf(stderr, "ERROR: Failed to read ROM file '%s'.\n", rom_filepath);
         fclose(rom_fp);
         free(*rom_data_ptr);
         *rom_data_ptr = NULL;
         return false;
     }
     fclose(rom_fp);
     verbose_printf("ROM loaded (%zu bytes).\n", *rom_size_ptr);
     return true;
 }

 /**
  * load_mapping_data() - Loads mapping file data.
  * @map_filepath: Path to the mapping file (can be NULL).
  * @mapping_table: Pointer to the mapping table structure.
  *
  * Return: true on success, false on failure.
  */
 bool
 load_mapping_data(const char *map_filepath, MappingTable *mapping_table)
 {
     init_mapping_table(mapping_table);
     if (map_filepath) {
         verbose_printf("Loading mappings (expecting 0-based segment index)...\n");
         if (!load_mappings(map_filepath, mapping_table)) {
             /* Error message already printed by load_mappings */
             return false;
         }
         verbose_printf("Loaded %zu mappings.\n", mapping_table->count);
     }
     return true;
 }


 /* --- Main Function --- */

 /**
  * print_usage() - Prints usage instructions to stderr.
  * @prog_name: Name of the executable (argv[0]).
  */
 void
 print_usage(const char *prog_name)
 {
     fprintf(stderr, "Usage: %s <rom_filepath> [-m <map_filepath>] [-i <message_index>] [-l|--list] [-q|--quiet] [-v|--verbose]\n", prog_name);
     fprintf(stderr, "Decodes Nortel Millennium VoiceWare ROM files (NEC uPD7759 ADPCM).\n");
     fprintf(stderr, "Uses 0-based segment indexing.\n");
     fprintf(stderr, "Options:\n");
     fprintf(stderr, "  <rom_filepath>      Path to the input ROM file.\n");
     fprintf(stderr, "  -m <map_filepath>   Path to the optional tab-delimited mapping file.\n");
     fprintf(stderr, "                      Format: SegIdx(0+)\\tMsgIdxInSeg(0+)\\tFilenameBase[\\tComment]\n");
     fprintf(stderr, "  -i <message_index>  Decode only the specified absolute message index (0-based).\n");
     fprintf(stderr, "                      (Ignored if -l or --list is specified).\n");
     fprintf(stderr, "  -l, --list          List messages in mapping file format (0-based SegIdx) to stdout\n");
     fprintf(stderr, "                      instead of decoding. Includes header comment '# ROM: <basename>\\n\\n'.\n");
     fprintf(stderr, "                      Uses tabs for padding to align comments (assuming %d char filename width & %d-space tabs).\n", LIST_FILENAME_ALIGN_WIDTH, TAB_WIDTH);
     fprintf(stderr, "                      Comments are prefixed with '#'. PCM messages are indicated,\n");
     fprintf(stderr, "                      avoiding duplication if '(PCM)' is already in map comment.\n");
     fprintf(stderr, "  -q, --quiet         Quiet mode. Suppress all informational output (stdout & stderr).\n" );
     fprintf(stderr, "                      Only errors are printed to stderr. Overrides -v.\n");
     fprintf(stderr, "  -v, --verbose       Enable verbose debugging output to stderr. Ignored if -q is used.\n");
 }

 /**
  * main() - Main entry point.
  * @argc: Argument count.
  * @argv: Argument vector.
  *
  * Return: EXIT_SUCCESS or EXIT_FAILURE.
  */
 int
 main(int argc, char *argv[])
 {
     const char *rom_filepath = NULL;
     const char *map_filepath = NULL;
     long target_message_idx = -1;
     const char *rom_basename;
     MappingTable mapping_table;
     size_t rom_size = 0;
     uint8_t *rom_data = NULL;
     int segment_index_0_based = 0;
     int absolute_msg_idx_counter = 0;
     bool target_found_and_processed = false;
     int exit_code = EXIT_SUCCESS;
     size_t segment_start;

     mapping_table.mappings = NULL; /* Ensure initialized for cleanup */
     mapping_table.count = 0;
     mapping_table.capacity = 0;

     /* --- Argument Parsing --- */
     if (!parse_arguments(argc, argv, &rom_filepath, &map_filepath,
                  &target_message_idx, &list_mode, &quiet_mode, &verbose_mode)) {
         /* Error or help message already printed */
         return (argc > 1 && (strcmp(argv[argc-1], "-h") == 0 || strcmp(argv[argc-1], "--help") == 0)) ? EXIT_SUCCESS : EXIT_FAILURE;
     }

     rom_basename = get_base_filename(rom_filepath);

     /* Print startup messages unless quiet */
     status_printf("Nortel Millennium VoiceWare Decoder (0-Based Segments)\n");
     /* Display Version/Commit Info */
     status_printf("Version: %s (%s)\n", GIT_TAG_NAME, GIT_COMMIT_HASH);
     status_printf("Input ROM: %s (Artist Tag: %s)\n", rom_filepath, rom_basename);
     if (map_filepath)
         status_printf("Mapping File: %s\n", map_filepath);
     if (list_mode)
         status_printf("Mode: Listing messages\n");
     else if (target_message_idx >= 0)
         status_printf("Mode: Decoding target message index %ld\n", target_message_idx);
     else
         status_printf("Mode: Decoding all messages\n");
     if (verbose_mode) /* verbose implies not quiet */
         printf("Verbose Mode: Enabled\n"); /* Use printf as verbose goes to stderr */


     /* --- Load Mappings --- */
     if (!load_mapping_data(map_filepath, &mapping_table)) {
         exit_code = EXIT_FAILURE;
         goto cleanup;
     }

     /* --- Load ROM Data --- */
     if (!load_rom_data(rom_filepath, &rom_data, &rom_size)) {
         exit_code = EXIT_FAILURE;
         goto cleanup;
     }

     /* --- Print List Header (if applicable) --- */
     if (list_mode && !quiet_mode) {
         printf("# ROM: %s\n\n", rom_basename);
     }

     /* --- Process Segments and Messages --- */
     for (segment_start = 0; segment_start < rom_size; segment_start += ROM_SEGMENT_SIZE, ++segment_index_0_based) {
         uint8_t last_message_index;
         uint32_t message_count_in_segment;
         size_t offset_table_start, offset_table_size;
         uint16_t *offset_table = NULL;
         uint32_t msg_idx_in_seg; /* Use unsigned to match message_count */
         bool segment_error = false;

         verbose_printf("Processing Segment %d (Offset 0x%zX)...\n", segment_index_0_based, segment_start);

         /* Check header */
         if (segment_start + 5 > rom_size) {
             if (segment_index_0_based > 0) {
                  verbose_printf("  INFO: Incomplete segment data at end of file. Stopping.\n");
             } else {
                  fprintf(stderr, "ERROR: ROM file too small for even one segment header.\n");
                  exit_code = EXIT_FAILURE;
             }
             break;
         }
         last_message_index = rom_data[segment_start];
         if (memcmp(rom_data + segment_start + 1, ROM_MAGIC, 4) != 0) {
             if (segment_index_0_based == 0) {
                 fprintf(stderr, "ERROR: Invalid magic number in first segment (Segment 0) header.\n");
                 exit_code = EXIT_FAILURE;
             } else {
                 verbose_printf("  INFO: Invalid magic number found at segment %d start. Assuming end of ROM data.\n", segment_index_0_based);
             }
             break;
         }

         message_count_in_segment = (uint32_t)last_message_index + 1;
         verbose_printf("  Segment Header OK: Last Message Index %u (%u messages)\n", last_message_index, message_count_in_segment);

         /* Check offset table size */
         offset_table_start = segment_start + 5;
         offset_table_size = message_count_in_segment * sizeof(uint16_t);
         if (offset_table_start + offset_table_size > rom_size ||
             offset_table_start + offset_table_size > segment_start + ROM_SEGMENT_SIZE) {
             fprintf(stderr, "ERROR: Offset table size (%zu bytes for %u messages) exceeds segment/ROM bounds for segment %d.\n",
                 offset_table_size, message_count_in_segment, segment_index_0_based);
             exit_code = EXIT_FAILURE;
             break;
         }

         /* Read offset table */
         offset_table = (uint16_t *)malloc(offset_table_size);
         if (!offset_table) {
              fprintf(stderr, "ERROR: Failed to allocate memory for offset table (segment %d).\n", segment_index_0_based);
              exit_code = EXIT_FAILURE;
              break;
         }
         for (uint32_t k = 0; k < message_count_in_segment; ++k)
             offset_table[k] = read_u16be(rom_data + offset_table_start + k * 2);
         verbose_printf("  Offset table read for %u messages.\n", message_count_in_segment);

         /* Process messages within the segment */
         for (msg_idx_in_seg = 0; msg_idx_in_seg < message_count_in_segment; ++msg_idx_in_seg) {
             HandleMessageResult result = handle_message_iteration(
                 rom_data, rom_size, segment_start, segment_index_0_based,
                 msg_idx_in_seg, absolute_msg_idx_counter + msg_idx_in_seg,
                 offset_table, message_count_in_segment,
                 &mapping_table, rom_basename,
                 list_mode, quiet_mode, target_message_idx);

             if (result == MSG_HANDLED_ERROR) {
                 exit_code = EXIT_FAILURE; /* Mark failure but continue loop to free table */
                 segment_error = true;
                 break; /* Stop processing messages in this segment */
             } else if (result == MSG_HANDLED_TARGET_FOUND) {
                 target_found_and_processed = true;
                 break; /* Stop processing messages in this segment */
             }
         } /* End message loop */

         free(offset_table);
         absolute_msg_idx_counter += message_count_in_segment;

         if (segment_error || target_found_and_processed)
             break; /* Stop processing segments */

     } /* End segment loop */

     /* Check if the target message was specified but not found (only in decode mode) */
     if (!list_mode && target_message_idx >= 0 && !target_found_and_processed && exit_code != EXIT_FAILURE) {
         fprintf(stderr, "ERROR: Target message index %ld not found in the ROM file.\n", target_message_idx);
         exit_code = EXIT_FAILURE;
     }

 cleanup:
     /* --- Cleanup --- */
     verbose_printf("Cleaning up...\n");
     free(rom_data);
     free_mapping_table(&mapping_table);

     status_printf("Processing finished with exit code %d.\n", exit_code);

     return exit_code;
 }
