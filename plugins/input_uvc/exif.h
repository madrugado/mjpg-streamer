#include <assert.h>
#include <time.h>
#include <linux/limits.h>
#include <sys/time.h>
#include <jpeglib.h> 

/* EXIF image data is always in TIFF format, even if embedded in another
 * file type. This consists of a constant header (TIFF file header,
 * IFD header) followed by the tags in the IFD and then the data
 * from any tags which do not fit inline in the IFD.
 *
 * The tags we write in the main IFD are:
 *  0x010E   Image description
 *  0x8769   Exif sub-IFD
 *  0x882A   Time zone of time stamps
 * and in the Exif sub-IFD:
 *  0x9000   Exif version
 *  0x9003   File date and time
 *  0x9291   File date and time subsecond info
 * But we omit any empty IFDs.
 */

#define TIFF_TAG_IMAGE_DESCRIPTION    0x010E
#define TIFF_TAG_DATETIME             0x0132
#define TIFF_TAG_EXIF_IFD             0x8769
#define TIFF_TAG_TZ_OFFSET            0x882A

#define EXIF_TAG_EXIF_VERSION         0x9000
#define EXIF_TAG_ORIGINAL_DATETIME    0x9003
#define EXIF_TAG_SUBJECT_AREA         0x9214
#define EXIF_TAG_TIFF_DATETIME_SS     0x9290
#define EXIF_TAG_ORIGINAL_DATETIME_SS 0x9291

#define TIFF_TYPE_ASCII  2  /* ASCII text */
#define TIFF_TYPE_USHORT 3  /* Unsigned 16-bit int */
#define TIFF_TYPE_LONG   4  /* Unsigned 32-bit int */
#define TIFF_TYPE_UNDEF  7  /* Byte blob */
#define TIFF_TYPE_SSHORT 8  /* Signed 16-bit int */

static const char exif_marker_start[14] = {
    'E', 'x', 'i', 'f', 0, 0,   /* EXIF marker signature */
    'M', 'M', 0, 42,            /* TIFF file header (big-endian) */
    0, 0, 0, 8,                 /* Offset to first toplevel IFD */
};
static const char exif_version_tag[12] = {
    0x90, 0x00,                 /* EXIF version tag, 0x9000 */
    0x00, 0x07,                 /* Data type 7 = "unknown" (raw byte blob) */
    0x00, 0x00, 0x00, 0x04,     /* Data length */
    0x30, 0x32, 0x32, 0x30      /* Inline data, EXIF version 2.2 */
};
static const char exif_subifd_tag[8] = {
    0x87, 0x69,                 /* EXIF Sub-IFD tag */
    0x00, 0x04,                 /* Data type 4 = uint32 */
    0x00, 0x00, 0x00, 0x01,     /* Number of values */
};
static const char exif_tzoffset_tag[12] = {
    0x88, 0x2A,                 /* TIFF/EP time zone offset tag */
    0x00, 0x08,                 /* Data type 8 = sint16 */
    0x00, 0x00, 0x00, 0x01,     /* Number of values */
    0, 0, 0, 0                  /* Dummy data */
};

static void put_uint16(JOCTET *buf, unsigned value)
{
    buf[0] = ( value & 0xFF00 ) >> 8;
    buf[1] = ( value & 0x00FF );
}
static void put_sint16(JOCTET *buf, int value)
{
    buf[0] = ( value & 0xFF00 ) >> 8;
    buf[1] = ( value & 0x00FF );
}
static void put_uint32(JOCTET *buf, unsigned value)
{
    buf[0] = ( value & 0xFF000000 ) >> 24;
    buf[1] = ( value & 0x00FF0000 ) >> 16;
    buf[2] = ( value & 0x0000FF00 ) >> 8;
    buf[3] = ( value & 0x000000FF );
}

struct tiff_writing {
    JOCTET * const base;
    JOCTET *buf;
    unsigned data_offset;
};

struct config {
	const char *exif_text;
};

struct context {
	struct config conf;
};

static void put_direntry(struct tiff_writing *into, const char *data, unsigned length)
{
    if (length <= 4) {
	/* Entries that fit in the directory entry are stored there */
	memset(into->buf, 0, 4);
	memcpy(into->buf, data, length);
    } else {
	/* Longer entries are stored out-of-line */
	unsigned offset = into->data_offset;
	while ((offset & 0x03) != 0) {  /* Alignment */
	    into->base[offset] = 0;
	    offset ++;
	}
	put_uint32(into->buf, offset);
	memcpy(into->base + offset, data, length);
	into->data_offset = offset + length;
    }
}
static void put_stringentry(struct tiff_writing *into, unsigned tag, const char *str, int with_nul)
{
    put_uint16(into->buf, tag);
    put_uint16(into->buf + 2, TIFF_TYPE_ASCII);
    unsigned stringlength = strlen(str) + (with_nul?1:0);
    put_uint32(into->buf + 4, stringlength);
    into->buf += 8;
    put_direntry(into, str, stringlength);
    into->buf += 4;
}

