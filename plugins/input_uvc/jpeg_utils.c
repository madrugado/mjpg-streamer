/*******************************************************************************
# Linux-UVC streaming input-plugin for MJPG-streamer                           #
#                                                                              #
# This package work with the Logitech UVC based webcams with the mjpeg feature #
#                                                                              #
#   Orginally Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard       #
#   Modifications Copyright (C) 2006  Gabriel A. Devenyi                       #
#   Modifications Copyright (C) 2007  Tom Stöveken                             #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; either version 2 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "v4l2uvc.h"
#include "exif.h"


/*
 * put_jpeg_exif writes the EXIF APP1 chunk to the jpeg file.
 * It must be called after jpeg_start_compress() but before
 * any image data is written by jpeg_write_scanlines().
 */
static void put_jpeg_exif(j_compress_ptr cinfo,
			  const struct context *cnt,
			  const struct timeval *time)
{
    /* description, datetime, and subtime are the values that are actually
     * put into the EXIF data
     */
    char *description, *datetime, *subtime;
    char datetime_buf[22], subtime_buf[5];

    struct tm *timestamp = NULL;
    if (time) 
	    timestamp = localtime(&(time->tv_sec));

    if (timestamp) {
	/* Exif requires this exact format */
	snprintf(datetime_buf, 21, "%04d:%02d:%02d %02d:%02d:%02d",
		 timestamp->tm_year + 1900,
		 timestamp->tm_mon + 1,
		 timestamp->tm_mday,
		 timestamp->tm_hour,
		 timestamp->tm_min,
		 timestamp->tm_sec);
	datetime = datetime_buf;
    } else {
	datetime = NULL;
    }

    /* Exif is not specified the format of subsectime, but since we have unixtime
     * we'll use 000 as format. */
    if (time) {
	suseconds_t subtimestamp = time->tv_usec - (time->tv_usec / 1000) * 1000;
	snprintf(subtime_buf, 4, "%03d", subtimestamp);	
	subtime = subtime_buf;
    } else
    	subtime = NULL;

    if (cnt->conf.exif_text) {
	description = malloc(PATH_MAX);
	mystrftime(cnt, description, PATH_MAX-1,
		   cnt->conf.exif_text,
		   timestamp, NULL, 0);
    } else {
	description = NULL;
    }

    /* Calculate an upper bound on the size of the APP1 marker so
     * we can allocate a buffer for it.
     */

    /* Count up the number of tags and max amount of OOL data */
    int ifd0_tagcount = 0;
    int ifd1_tagcount = 0;
    unsigned datasize = 0;

    if (description) {
	ifd0_tagcount ++;
	datasize += 5 + strlen(description); /* Add 5 for NUL and alignment */
    }
    if (datetime) {
	/* We write this to both the TIFF datetime tag (which most programs
	 * treat as "last-modified-date") and the EXIF "time of creation of
	 * original image" tag (which many programs ignore). This is
	 * redundant but seems to be the thing to do.
	 */
	ifd0_tagcount ++;
	ifd1_tagcount ++;
	/* We also write the timezone-offset tag in IFD0 */
	ifd0_tagcount ++;
	/* It would be nice to use the same offset for both tags' values,
	 * but I don't want to write the bookkeeping for that right now */
	datasize += 2 * (5 + strlen(datetime));
    }
    if (subtime) {
	ifd1_tagcount ++;
	datasize += 5 + strlen(subtime);
    }
    if (ifd1_tagcount > 0) {
	/* If we're writing the Exif sub-IFD, account for the
	 * two tags that requires */
	ifd0_tagcount ++; /* The tag in IFD0 that points to IFD1 */
	ifd1_tagcount ++; /* The EXIF version tag */
    }

    /* Each IFD takes 12 bytes per tag, plus six more (the tag count and the
     * pointer to the next IFD, always zero in our case)
     */
    unsigned int ifds_size =
	( ifd1_tagcount > 0 ? ( 12 * ifd1_tagcount + 6 ) : 0 ) +
	( ifd0_tagcount > 0 ? ( 12 * ifd0_tagcount + 6 ) : 0 );

    if (ifds_size == 0) {
	/* We're not actually going to write any information. */
	return;
    }

    unsigned int buffer_size = 6 /* EXIF marker signature */ +
                               8 /* TIFF file header */ +
                               ifds_size /* the tag directories */ +
                               datasize;

    JOCTET *marker = malloc(buffer_size);
    memcpy(marker, exif_marker_start, 14); /* EXIF and TIFF headers */
    struct tiff_writing writing = (struct tiff_writing){
	.base = marker + 6, /* base address for intra-TIFF offsets */
	.buf = marker + 14, /* current write position */
	.data_offset = 8 + ifds_size, /* where to start storing data */
    };

    /* Write IFD 0 */
    /* Note that tags are stored in numerical order */
    put_uint16(writing.buf, ifd0_tagcount);
    writing.buf += 2;
    if (description)
	put_stringentry(&writing, TIFF_TAG_IMAGE_DESCRIPTION, description, 0);
    if (datetime)
	put_stringentry(&writing, TIFF_TAG_DATETIME, datetime, 1);
    if (ifd1_tagcount > 0) {
	/* Offset of IFD1 - TIFF header + IFD0 size. */
	unsigned ifd1_offset = 8 + 6 + ( 12 * ifd0_tagcount );
	memcpy(writing.buf, exif_subifd_tag, 8);
	put_uint32(writing.buf + 8, ifd1_offset);
	writing.buf += 12;
    }
    if (datetime) {
        memcpy(writing.buf, exif_tzoffset_tag, 12);
        put_sint16(writing.buf+8, timestamp->tm_gmtoff / 3600);
        writing.buf += 12;
    }
    put_uint32(writing.buf, 0); /* Next IFD offset = 0 (no next IFD) */
    writing.buf += 4;

    /* Write IFD 1 */
    if (ifd1_tagcount > 0) {
	/* (remember that the tags in any IFD must be in numerical order
	 * by tag) */
	put_uint16(writing.buf, ifd1_tagcount);
	memcpy(writing.buf + 2, exif_version_tag, 12); /* tag 0x9000 */
	writing.buf += 14;

	if (datetime)
	    put_stringentry(&writing, EXIF_TAG_ORIGINAL_DATETIME, datetime, 1);
	if (subtime)
	    put_stringentry(&writing, EXIF_TAG_ORIGINAL_DATETIME_SS, subtime, 0);

	put_uint32(writing.buf, 0); /* Next IFD = 0 (no next IFD) */
	writing.buf += 4;
    }

    /* We should have met up with the OOL data */
    assert( (writing.buf - writing.base) == 8 + ifds_size );

    /* The buffer is complete; write it out */
    unsigned marker_len = 6 + writing.data_offset;

    /* assert we didn't underestimate the original buffer size */
    assert(marker_len <= buffer_size);

    /* EXIF data lives in a JPEG APP1 marker */
    jpeg_write_marker(cinfo, JPEG_APP0 + 1, marker, marker_len);

    if (description)
	free(description);
    free(marker);
}

#define OUTPUT_BUF_SIZE  4096

typedef struct {
    struct jpeg_destination_mgr pub; /* public fields */

    JOCTET * buffer;    /* start of buffer */

    unsigned char *outbuffer;
    int outbuffer_size;
    unsigned char *outbuffer_cursor;
    int *written;

} mjpg_destination_mgr;

typedef mjpg_destination_mgr * mjpg_dest_ptr;

/******************************************************************************
Description.:
Input Value.:
Return Value:
******************************************************************************/
METHODDEF(void) init_destination(j_compress_ptr cinfo)
{
    mjpg_dest_ptr dest = (mjpg_dest_ptr) cinfo->dest;

    /* Allocate the output buffer --- it will be released when done with image */
    dest->buffer = (JOCTET *)(*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_IMAGE, OUTPUT_BUF_SIZE * sizeof(JOCTET));

    *(dest->written) = 0;

    dest->pub.next_output_byte = dest->buffer;
    dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;
}

/******************************************************************************
Description.: called whenever local jpeg buffer fills up
Input Value.:
Return Value:
******************************************************************************/
METHODDEF(boolean) empty_output_buffer(j_compress_ptr cinfo)
{
    mjpg_dest_ptr dest = (mjpg_dest_ptr) cinfo->dest;

    memcpy(dest->outbuffer_cursor, dest->buffer, OUTPUT_BUF_SIZE);
    dest->outbuffer_cursor += OUTPUT_BUF_SIZE;
    *(dest->written) += OUTPUT_BUF_SIZE;

    dest->pub.next_output_byte = dest->buffer;
    dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;

    return TRUE;
}

/******************************************************************************
Description.: called by jpeg_finish_compress after all data has been written.
              Usually needs to flush buffer.
Input Value.:
Return Value:
******************************************************************************/
METHODDEF(void) term_destination(j_compress_ptr cinfo)
{
    mjpg_dest_ptr dest = (mjpg_dest_ptr) cinfo->dest;
    size_t datacount = OUTPUT_BUF_SIZE - dest->pub.free_in_buffer;

    /* Write any data remaining in the buffer */
    memcpy(dest->outbuffer_cursor, dest->buffer, datacount);
    dest->outbuffer_cursor += datacount;
    *(dest->written) += datacount;
}

/******************************************************************************
Description.: Prepare for output to a stdio stream.
Input Value.: buffer is the already allocated buffer memory that will hold
              the compressed picture. "size" is the size in bytes.
Return Value: -
******************************************************************************/
GLOBAL(void) dest_buffer(j_compress_ptr cinfo, unsigned char *buffer, int size, int *written)
{
    mjpg_dest_ptr dest;

    if(cinfo->dest == NULL) {
        cinfo->dest = (struct jpeg_destination_mgr *)(*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(mjpg_destination_mgr));
    }

    dest = (mjpg_dest_ptr) cinfo->dest;
    dest->pub.init_destination = init_destination;
    dest->pub.empty_output_buffer = empty_output_buffer;
    dest->pub.term_destination = term_destination;
    dest->outbuffer = buffer;
    dest->outbuffer_size = size;
    dest->outbuffer_cursor = buffer;
    dest->written = written;
}

/******************************************************************************
Description.: yuv2jpeg function is based on compress_yuyv_to_jpeg written by
              Gabriel A. Devenyi.
              It uses the destination manager implemented above to compress
              YUYV data to JPEG. Most other implementations use the
              "jpeg_stdio_dest" from libjpeg, which can not store compressed
              pictures to memory instead of a file.
Input Value.: video structure from v4l2uvc.c/h, destination buffer and buffersize
              the buffer must be large enough, no error/size checking is done!
Return Value: the buffer will contain the compressed data
******************************************************************************/
int compress_yuyv_to_jpeg(struct vdIn *vd, unsigned char *buffer, int size, int quality)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char *line_buffer, *yuyv;
    int z;
    static int written;

    line_buffer = calloc(vd->width * 3, 1);
    yuyv = vd->framebuffer;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    /* jpeg_stdio_dest (&cinfo, file); */
    dest_buffer(&cinfo, buffer, size, &written);

    cinfo.image_width = vd->width;
    cinfo.image_height = vd->height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    jpeg_start_compress(&cinfo, TRUE);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    put_jpeg_exif(&cinfo, NULL, &tv);

    z = 0;
    while(cinfo.next_scanline < vd->height) {
        int x;
        unsigned char *ptr = line_buffer;

        for(x = 0; x < vd->width; x++) {
            int r, g, b;
            int y, u, v;

            if(!z)
                y = yuyv[0] << 8;
            else
                y = yuyv[2] << 8;
            u = yuyv[1] - 128;
            v = yuyv[3] - 128;

            r = (y + (359 * v)) >> 8;
            g = (y - (88 * u) - (183 * v)) >> 8;
            b = (y + (454 * u)) >> 8;

            *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
            *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
            *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

            if(z++) {
                z = 0;
                yuyv += 4;
            }
        }

        row_pointer[0] = line_buffer;
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    free(line_buffer);

    return (written);
}

