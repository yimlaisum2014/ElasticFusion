/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef TOOLS_JPEGLOADER_H_
#define TOOLS_JPEGLOADER_H_

extern "C"
{
#include "jpeglib.h"
}

#include <stdio.h>
#include <string>

static void jpegFail(j_common_ptr cinfo)
{
    assert(false && "JPEG decoding error!");
}

static void doNothing(j_decompress_ptr)
{

}

class JPEGLoader
{
    public:
        JPEGLoader()
        {}

        void readData(unsigned char * src, const int numBytes, unsigned char * data)
        {
            jpeg_decompress_struct cinfo; // IJG JPEG codec structure

            jpeg_error_mgr errorMgr;

            errorMgr.error_exit = jpegFail;

            cinfo.err = jpeg_std_error(&errorMgr);

            jpeg_create_decompress(&cinfo);

            jpeg_source_mgr srcMgr;

            cinfo.src = &srcMgr;

            // Prepare for suspending reader
            srcMgr.init_source = doNothing;
            srcMgr.resync_to_restart = jpeg_resync_to_restart;
            srcMgr.term_source = doNothing;
            srcMgr.next_input_byte = src;
            srcMgr.bytes_in_buffer = numBytes;

            jpeg_read_header(&cinfo, TRUE);

            jpeg_calc_output_dimensions(&cinfo);

            jpeg_start_decompress(&cinfo);

            int width = cinfo.output_width;
            int height = cinfo.output_height;

            JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo, JPOOL_IMAGE, width * 4, 1);

            for(; height--; data += (width * 3))
            {
                std::cout << "height " << height << std::endl;
                jpeg_read_scanlines(&cinfo, buffer, 1);

                unsigned char * bgr = (unsigned char *)buffer[0];
                unsigned char * rgb = (unsigned char *)data;

                for(int i = 0; i < width; i++, bgr += 3, rgb += 3)
                {
                    unsigned char t0 = bgr[0], t1 = bgr[1], t2 = bgr[2];
                    rgb[2] = t0; rgb[1] = t1; rgb[0] = t2;
                }
            }

            jpeg_finish_decompress(&cinfo);

            jpeg_destroy_decompress(&cinfo);
        }

        static void
        init_source (j_decompress_ptr cinfo)
        {
        }

        static boolean
        fill_input_buffer (j_decompress_ptr cinfo)
        {
        //    fprintf (stderr, "Error: JPEG decompressor ran out of buffer space\n");
            return TRUE;
        }

        static void
        skip_input_data (j_decompress_ptr cinfo, long num_bytes)
        {
            cinfo->src->next_input_byte += num_bytes;
            cinfo->src->bytes_in_buffer -= num_bytes;
        }

        static void
        term_source (j_decompress_ptr cinfo)
        {
        }

        static void 
        jpeg_err_emit_message(j_common_ptr cinfo, int msg_level)
        {
            // suppress warnings and errors
        }

        int jpeg_decompress_8u (const uint8_t * src, int src_size, 
            uint8_t * dest, int width, int height, int stride, J_COLOR_SPACE ocs)
        {
            struct jpeg_decompress_struct cinfo;
            struct jpeg_error_mgr jerr;
            struct jpeg_source_mgr jsrc;

            cinfo.err = jpeg_std_error (&jerr);
            //jerr.emit_message = jpeg_err_emit_message;
            jpeg_create_decompress (&cinfo);

            jsrc.next_input_byte = src;
            jsrc.bytes_in_buffer = src_size;
            jsrc.init_source = init_source;
            jsrc.fill_input_buffer = fill_input_buffer;
            jsrc.skip_input_data = skip_input_data;
            jsrc.resync_to_restart = jpeg_resync_to_restart;
            jsrc.term_source = term_source;
            cinfo.src = &jsrc;

            jpeg_read_header (&cinfo, TRUE);
            cinfo.out_color_space = ocs;
            jpeg_start_decompress (&cinfo);

            if (cinfo.output_height != height || cinfo.output_width != width) {
                fprintf (stderr, "Error: Buffer was %dx%d but JPEG image is %dx%d\n",
                        width, height, cinfo.output_width, cinfo.output_height);
                jpeg_destroy_decompress (&cinfo);
                return -1;
            }

            std::cout << "about to read jpeg_read_scanlines" << std::endl;
            while (cinfo.output_scanline < height) {
                uint8_t * row = dest + cinfo.output_scanline * stride;
                jpeg_read_scanlines (&cinfo, &row, 1);
            }
            std::cout << "finished read jpeg_read_scanlines" << std::endl;
            try {
                //jpeg_finish_decompress (&cinfo);
            }
            catch(...) {
                std::cout << "caught jpeg_finish_decompress" << std::endl;
            }
            std::cout << "finished jpeg_finish_decompress" << std::endl;
            //jpeg_destroy_decompress (&cinfo);
            std::cout << "finished jpeg_destroy_decompress" << std::endl;
            return 0;
        }
};


#endif /* TOOLS_JPEGLOADER_H_ */
