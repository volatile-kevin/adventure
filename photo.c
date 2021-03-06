/*									tab:8
 *
 * photo.c - photo display functions
 *
 * "Copyright (c) 2011 by Steven S. Lumetta."
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice and the following
 * two paragraphs appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE AUTHOR OR THE UNIVERSITY OF ILLINOIS BE LIABLE TO
 * ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT  OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHOR AND/OR THE UNIVERSITY OF ILLINOIS HAS BEEN ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE AUTHOR AND THE UNIVERSITY OF ILLINOIS SPECIFICALLY DISCLAIM ANY
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
 * PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND NEITHER THE AUTHOR NOR
 * THE UNIVERSITY OF ILLINOIS HAS ANY OBLIGATION TO PROVIDE MAINTENANCE,
 * SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * Author:	    Steve Lumetta
 * Version:	    3
 * Creation Date:   Fri Sep  9 21:44:10 2011
 * Filename:	    photo.c
 * History:
 *	SL	1	Fri Sep  9 21:44:10 2011
 *		First written (based on mazegame code).
 *	SL	2	Sun Sep 11 14:57:59 2011
 *		Completed initial implementation of functions.
 *	SL	3	Wed Sep 14 21:49:44 2011
 *		Cleaned up code for distribution.
 */


#include <string.h>

#include "assert.h"
#include "modex.h"
#include "photo.h"
#include "photo_headers.h"
#include "world.h"


/* types local to this file (declared in types.h) */

/*
 * A room photo.  Note that you must write the code that selects the
 * optimized palette colors and fills in the pixel data using them as
 * well as the code that sets up the VGA to make use of these colors.
 * Pixel data are stored as one-byte values starting from the upper
 * left and traversing the top row before returning to the left of
 * the second row, and so forth.  No padding should be used.
 */
#define LFOURSIZE 4096
#define LTWOSIZE 64
#define LFOURFINALSIZE 128
#define PALETTESIZE 192
/*
 * An object image.  The code for managing these images has been given
 * to you.  The data are simply loaded from a file, where they have
 * been stored as 2:2:2-bit RGB values (one byte each), including
 * transparent pixels (value OBJ_CLR_TRANSP).  As with the room photos,
 * pixel data are stored as one-byte values starting from the upper
 * left and traversing the top row before returning to the left of the
 * second row, and so forth.  No padding is used.
 */
struct image_t {
    photo_header_t hdr;			/* defines height and width */
    uint8_t*       img;                 /* pixel data               */
};

typedef struct colorData_t4 {
    unsigned int       saveIndex;                 /* pixel data               */
    unsigned int       frequency;                 /* pixel data               */

    unsigned int       total_R4;                 /* pixel data               */
    unsigned int       total_G4;                 /* pixel data               */
    unsigned int       total_B4;                 /* pixel data               */

    unsigned int       avg_R4;                 /* pixel data               */
    unsigned int       avg_G4;                 /* pixel data               */
    unsigned int       avg_B4;                 /* pixel data               */

} colorDataL4;

typedef struct colorData_t2 {
    unsigned int       saveIndex;                 /* pixel data               */
    unsigned int       frequency;                 /* pixel data               */

    unsigned int       total_R2;                 /* pixel data               */
    unsigned int       total_G2;                 /* pixel data               */
    unsigned int       total_B2;                 /* pixel data               */

    unsigned int       avg_R2;                 /* pixel data               */
    unsigned int       avg_G2;                 /* pixel data               */
    unsigned int       avg_B2;                 /* pixel data               */
} colorDataL2;

/* file-scope variables */

/*
 * The room currently shown on the screen.  This value is not known to
 * the mode X code, but is needed when filling buffers in callbacks from
 * that code (fill_horiz_buffer/fill_vert_buffer).  The value is set
 * by calling prep_room.
 */
static const room_t* cur_room = NULL;


/*
 * fill_horiz_buffer
 *   DESCRIPTION: Given the (x,y) map pixel coordinate of the leftmost
 *                pixel of a line to be drawn on the screen, this routine
 *                produces an image of the line.  Each pixel on the line
 *                is represented as a single byte in the image.
 *
 *                Note that this routine draws both the room photo and
 *                the objects in the room.
 *
 *   INPUTS: (x,y) -- leftmost pixel of line to be drawn
 *   OUTPUTS: buf -- buffer holding image data for the line
 *   RETURN VALUE: none
 *   SIDE EFFECTS: none
 */
void
fill_horiz_buffer (int x, int y, unsigned char buf[SCROLL_X_DIM])
{
    int            idx;   /* loop index over pixels in the line          */
    object_t*      obj;   /* loop index over objects in the current room */
    int            imgx;  /* loop index over pixels in object image      */
    int            yoff;  /* y offset into object image                  */
    uint8_t        pixel; /* pixel from object image                     */
    const photo_t* view;  /* room photo                                  */
    int32_t        obj_x; /* object x position                           */
    int32_t        obj_y; /* object y position                           */
    const image_t* img;   /* object image                                */

    /* Get pointer to current photo of current room. */
    view = room_photo (cur_room);

    /* Loop over pixels in line. */
    for (idx = 0; idx < SCROLL_X_DIM; idx++) {
        buf[idx] = (0 <= x + idx && view->hdr.width > x + idx ?
		    view->img[view->hdr.width * y + x + idx] : 0);
    }

    /* Loop over objects in the current room. */
    for (obj = room_contents_iterate (cur_room); NULL != obj;
    	 obj = obj_next (obj)) {
	obj_x = obj_get_x (obj);
	obj_y = obj_get_y (obj);
	img = obj_image (obj);

        /* Is object outside of the line we're drawing? */
	if (y < obj_y || y >= obj_y + img->hdr.height ||
	    x + SCROLL_X_DIM <= obj_x || x >= obj_x + img->hdr.width) {
	    continue;
	}

	/* The y offset of drawing is fixed. */
	yoff = (y - obj_y) * img->hdr.width;

	/*
	 * The x offsets depend on whether the object starts to the left
	 * or to the right of the starting point for the line being drawn.
	 */
	if (x <= obj_x) {
	    idx = obj_x - x;
	    imgx = 0;
	} else {
	    idx = 0;
	    imgx = x - obj_x;
	}

	/* Copy the object's pixel data. */
	for (; SCROLL_X_DIM > idx && img->hdr.width > imgx; idx++, imgx++) {
	    pixel = img->img[yoff + imgx];

	    /* Don't copy transparent pixels. */
	    if (OBJ_CLR_TRANSP != pixel) {
		buf[idx] = pixel;
	    }
	}
    }
}


/*
 * fill_vert_buffer
 *   DESCRIPTION: Given the (x,y) map pixel coordinate of the top pixel of
 *                a vertical line to be drawn on the screen, this routine
 *                produces an image of the line.  Each pixel on the line
 *                is represented as a single byte in the image.
 *
 *                Note that this routine draws both the room photo and
 *                the objects in the room.
 *
 *   INPUTS: (x,y) -- top pixel of line to be drawn
 *   OUTPUTS: buf -- buffer holding image data for the line
 *   RETURN VALUE: none
 *   SIDE EFFECTS: none
 */
void
fill_vert_buffer (int x, int y, unsigned char buf[SCROLL_Y_DIM])
{
    int            idx;   /* loop index over pixels in the line          */
    object_t*      obj;   /* loop index over objects in the current room */
    int            imgy;  /* loop index over pixels in object image      */
    int            xoff;  /* x offset into object image                  */
    uint8_t        pixel; /* pixel from object image                     */
    const photo_t* view;  /* room photo                                  */
    int32_t        obj_x; /* object x position                           */
    int32_t        obj_y; /* object y position                           */
    const image_t* img;   /* object image                                */

    /* Get pointer to current photo of current room. */
    view = room_photo (cur_room);

    /* Loop over pixels in line. */
    for (idx = 0; idx < SCROLL_Y_DIM; idx++) {
        buf[idx] = (0 <= y + idx && view->hdr.height > y + idx ?
		    view->img[view->hdr.width * (y + idx) + x] : 0);
    }

    /* Loop over objects in the current room. */
    for (obj = room_contents_iterate (cur_room); NULL != obj;
    	 obj = obj_next (obj)) {
	obj_x = obj_get_x (obj);
	obj_y = obj_get_y (obj);
	img = obj_image (obj);

        /* Is object outside of the line we're drawing? */
	if (x < obj_x || x >= obj_x + img->hdr.width ||
	    y + SCROLL_Y_DIM <= obj_y || y >= obj_y + img->hdr.height) {
	    continue;
	}

	/* The x offset of drawing is fixed. */
	xoff = x - obj_x;

	/*
	 * The y offsets depend on whether the object starts below or
	 * above the starting point for the line being drawn.
	 */
	if (y <= obj_y) {
	    idx = obj_y - y;
	    imgy = 0;
	} else {
	    idx = 0;
	    imgy = y - obj_y;
	}

	/* Copy the object's pixel data. */
	for (; SCROLL_Y_DIM > idx && img->hdr.height > imgy; idx++, imgy++) {
	    pixel = img->img[xoff + img->hdr.width * imgy];

	    /* Don't copy transparent pixels. */
	    if (OBJ_CLR_TRANSP != pixel) {
		buf[idx] = pixel;
	    }
	}
    }
}


/*
 * image_height
 *   DESCRIPTION: Get height of object image in pixels.
 *   INPUTS: im -- object image pointer
 *   OUTPUTS: none
 *   RETURN VALUE: height of object image im in pixels
 *   SIDE EFFECTS: none
 */
uint32_t
image_height (const image_t* im)
{
    return im->hdr.height;
}


/*
 * image_width
 *   DESCRIPTION: Get width of object image in pixels.
 *   INPUTS: im -- object image pointer
 *   OUTPUTS: none
 *   RETURN VALUE: width of object image im in pixels
 *   SIDE EFFECTS: none
 */
uint32_t
image_width (const image_t* im)
{
    return im->hdr.width;
}

/*
 * photo_height
 *   DESCRIPTION: Get height of room photo in pixels.
 *   INPUTS: p -- room photo pointer
 *   OUTPUTS: none
 *   RETURN VALUE: height of room photo p in pixels
 *   SIDE EFFECTS: none
 */
uint32_t
photo_height (const photo_t* p)
{
    return p->hdr.height;
}


/*
 * photo_width
 *   DESCRIPTION: Get width of room photo in pixels.
 *   INPUTS: p -- room photo pointer
 *   OUTPUTS: none
 *   RETURN VALUE: width of room photo p in pixels
 *   SIDE EFFECTS: none
 */
uint32_t
photo_width (const photo_t* p)
{
    return p->hdr.width;
}


/*
 * prep_room
 *   DESCRIPTION: Prepare a new room for display. Calls room_photo to get the photo_t struct of the room
 *                             Then pass that struct into fill_palette_mode_x_OCT to actually send to VGA
 *   INPUTS: r -- pointer to the new room
 *   OUTPUTS: none
 *   RETURN VALUE: none
 *   SIDE EFFECTS: changes recorded cur_room for this file
 */
void
prep_room (const room_t* r)
{
    /* Record the current room. */
    cur_room = r;
    photo_t* p = room_photo(r);
    // this function in modex.c sends the palette to VGA
    fill_palette_mode_x_OCT((unsigned int*)p->palette);
}


/*
 * read_obj_image
 *   DESCRIPTION: Read size and pixel data in 2:2:2 RGB format from a
 *                photo file and create an image structure from it.
 *   INPUTS: fname -- file name for input
 *   OUTPUTS: none
 *   RETURN VALUE: pointer to newly allocated photo on success, or NULL
 *                 on failure
 *   SIDE EFFECTS: dynamically allocates memory for the image
 */
image_t*
read_obj_image (const char* fname)
{
    FILE*    in;		/* input file               */
    image_t* img = NULL;	/* image structure          */
    uint16_t x;			/* index over image columns */
    uint16_t y;			/* index over image rows    */
    uint8_t  pixel;		/* one pixel from the file  */

    /*
     * Open the file, allocate the structure, read the header, do some
     * sanity checks on it, and allocate space to hold the image pixels.
     * If anything fails, clean up as necessary and return NULL.
     */
    if (NULL == (in = fopen (fname, "r+b")) ||
	NULL == (img = malloc (sizeof (*img))) ||
	NULL != (img->img = NULL) || /* false clause for initialization */
	1 != fread (&img->hdr, sizeof (img->hdr), 1, in) ||
	MAX_OBJECT_WIDTH < img->hdr.width ||
	MAX_OBJECT_HEIGHT < img->hdr.height ||
	NULL == (img->img = malloc
		 (img->hdr.width * img->hdr.height * sizeof (img->img[0])))) {
	if (NULL != img) {
	    if (NULL != img->img) {
	        free (img->img);
	    }
	    free (img);
	}
	if (NULL != in) {
	    (void)fclose (in);
	}
	return NULL;
    }

    /*
     * Loop over rows from bottom to top.  Note that the file is stored
     * in this order, whereas in memory we store the data in the reverse
     * order (top to bottom).
     */
    for (y = img->hdr.height; y-- > 0; ) {

	/* Loop over columns from left to right. */
	for (x = 0; img->hdr.width > x; x++) {

	    /*
	     * Try to read one 8-bit pixel.  On failure, clean up and
	     * return NULL.
	     */
	    if (1 != fread (&pixel, sizeof (pixel), 1, in)) {
		free (img->img);
		free (img);
	        (void)fclose (in);
		return NULL;
	    }

	    /* Store the pixel in the image data. */
	    img->img[img->hdr.width * y + x] = pixel;
	}
    }

    /* All done.  Return success. */
    (void)fclose (in);
    return img;
}


/*
 * read_photo
 *   DESCRIPTION: Read size and pixel data in 5:6:5 RGB format from a
 *                photo file and create a photo structure from it.
 *                Code provided simply maps to 2:2:2 RGB.  You must
 *                replace this code with palette color selection, and
 *                must map the image pixels into the palette colors that
 *                you have defined.
 *   EDIT: Now reads pixel data and maps to 4:4:4 RGB as well as 2:2:2. these colors will be
 *              mapped to the palette which the VGA reads
 *   INPUTS: fname -- file name for input
 *   OUTPUTS: none
 *   RETURN VALUE: pointer to newly allocated photo on success, or NULL
 *                 on failure
 *   SIDE EFFECTS: dynamically allocates memory for the photo
 */
photo_t*
read_photo (const char* fname)
{
    FILE*    in;	/* input file               */
    photo_t* p = NULL;	/* photo structure          */
    uint16_t x;		/* index over image columns */
    uint16_t y;		/* index over image rows    */
    uint16_t pixel;	/* one pixel from the file  */
    int palette_idx[LFOURSIZE];
    // fill this array with -1 to differentiate from actual data
    memset(palette_idx, -1, LFOURSIZE*sizeof(int));
    /*
     * Open the file, allocate the structure, read the header, do some
     * sanity checks on it, and allocate space to hold the photo pixels.
     * If anything fails, clean up as necessary and return NULL.
     */
    if (NULL == (in = fopen (fname, "r+b")) ||
	NULL == (p = malloc (sizeof (*p))) ||
	NULL != (p->img = NULL) || /* false clause for initialization */
	1 != fread (&p->hdr, sizeof (p->hdr), 1, in) ||
	MAX_PHOTO_WIDTH < p->hdr.width ||
	MAX_PHOTO_HEIGHT < p->hdr.height ||
	NULL == (p->img = malloc
		 (p->hdr.width * p->hdr.height * sizeof (p->img[0])))) {
	if (NULL != p) {
	    if (NULL != p->img) {
	        free (p->img);
	    }
	    free (p);
	}
	if (NULL != in) {
	    (void)fclose (in);
	}
	return NULL;
    }

    /*
     * Loop over rows from bottom to top.  Note that the file is stored
     * in this order, whereas in memory we store the data in the reverse
     * order (top to bottom).
     */
    colorDataL4 colorData4[LFOURSIZE];
    colorDataL2 colorData2[LTWOSIZE];
    memset(colorData4, 0, LFOURSIZE * sizeof(colorData4[0]));
    memset(colorData2, 0, LTWOSIZE * sizeof(colorData2[0]));
    for (y = p->hdr.height; y-- > 0; ) {
	/* Loop over columns from left to right. */
	     for (x = 0; p->hdr.width > x; x++) {
	    /*
	     * Try to read one 16-bit pixel.  On failure, clean up and
	     * return NULL.
	     */
	    if (1 != fread (&pixel, sizeof (pixel), 1, in)) {
    		free (p->img);
    		free (p);
    	  (void)fclose (in);
    		return NULL;
	    }
	    /*
	     * 16-bit pixel is coded as 5:6:5 RGB (5 bits red, 6 bits green,
	     * and 6 bits blue).  We change to 2:2:2, which we've set for the
	     * game objects.  You need to use the other 192 palette colors
	     * to specialize the appearance of each photo.
	     *
	     * In this code, you need to calculate the p->palette values,
	     * which encode 6-bit RGB as arrays of three uint8_t's.  When
	     * the game puts up a photo, you should then change the palette
	     * to match the colors needed for that photo.
	     */


      // get 4 MSB of each color in each pixel by
      // 1. right shift by 12 to isolate 4 MSB from R
      // 2. right shift by 7 and AND with 1111 = 0xF to isolate 4 MSB from G
      // 3. right shift by 1 and AND with 1111 = 0xF to isolate 4 MSB from B
      // 4. right shift 1. by 8, 2. by 4 and OR them all together to get 4:4:4
      p->img[p->hdr.width * y + x] = (((pixel >> 12) << 8) |
					    (((pixel >> 7) & 0xF) << 4) |
					    ((pixel >> 1) & 0xF));
      //save it into a variable to be sent to the struct
      int bitIndex4 =  (((pixel >> 12) << 8) |
					    (((pixel >> 7) & 0xF) << 4) |
					    ((pixel >> 1) & 0xF));

      // fill the struct array with RGB, index, and add to count
      colorData4[bitIndex4].frequency += 1;
      colorData4[bitIndex4].saveIndex = bitIndex4;
      // right shift pixel by 11 to get 5 bits of R
      // left shift it by 1 to make it a 6 bit number
      colorData4[bitIndex4].total_R4 += (pixel >> 11) << 1;
      // right shift pixel by 5 and AND with 111111 = 0x3F to isolate 6 bits of G
      colorData4[bitIndex4].total_G4 += (pixel >> 5) & 0x3F;
      // right shift pixel by 11 and AND with 11111 = 0x1F to get 5 bits of B
      // left shift it by 1 to make it a 6 bit number
      colorData4[bitIndex4].total_B4 += (pixel & 0x1F) << 1;
	    }
    }
    // sort by most populous, descending order
    qsort(colorData4, LFOURSIZE, sizeof(colorData4[0]), cmpfunc);
    // get the average of each bit string, only for most populated 128
    int i;
    int j;
  for(i = 0; i < LFOURFINALSIZE; i++){
    if(colorData4[i].frequency != 0){
      colorData4[i].avg_R4 = (colorData4[i].total_R4) / colorData4[i].frequency;
      colorData4[i].avg_G4 = colorData4[i].total_G4 / colorData4[i].frequency;
      colorData4[i].avg_B4 = (colorData4[i].total_B4) / colorData4[i].frequency;
    }
  }
// fill level 2 tree with level 4 nodes that are not in the most populous 128
for(i = LFOURFINALSIZE; i < LFOURSIZE; i++){
  // CONVERT L4 to L2 Index by:
  // 1. right shift L4 idx by 10 and AND with 11 = 3 to isolate 2 MSB of R
  // 2. right shift L4 idx by 6 and AND with 11 = 3 to isolate 2 MSB of G
  // 3. right shift L4 idx by 2 and AND with 11 = 3 to isolate 2 MSB of R
  // 4. left shift 1. by 4 and 2. by 2, OR them all together to get L2 index
  int idxL2 = ((((colorData4[i].saveIndex >> 10) & 3) << 4) | (((colorData4[i].saveIndex >> 6) & 3) << 2) | ((colorData4[i].saveIndex >> 2) & 3));
  colorData2[idxL2].frequency += colorData4[i].frequency;
  colorData2[idxL2].total_R2 += colorData4[i].total_R4;
  colorData2[idxL2].total_G2 += colorData4[i].total_G4;
  colorData2[idxL2].total_B2 += colorData4[i].total_B4;
}
// calculate averages for level 2
for(i = 0; i < LTWOSIZE; i++){
      if(colorData2[i].frequency != 0){
        colorData2[i].avg_R2 = (colorData2[i].total_R2) / colorData2[i].frequency;
        colorData2[i].avg_G2 = colorData2[i].total_G2 / colorData2[i].frequency;
        colorData2[i].avg_B2 = (colorData2[i].total_B2) / colorData2[i].frequency;
      }
    }
// put level 4 and level 2 nodes in the palette to get sent to VGA
  for(i = 0; i < PALETTESIZE; i++){
    for(j = 0; j < 3; j++){
      if(i < LFOURFINALSIZE){
        // array that keeps track of index so we don't have to loop again in the next iteration through pixels
        palette_idx[colorData4[i].saveIndex] = i;
        switch(j){
          case 0:
            p->palette[i][j] = colorData4[i].avg_R4;
            break;
          case 1:
            p->palette[i][j] = colorData4[i].avg_G4;
            break;
          case 2:
            p->palette[i][j] = colorData4[i].avg_B4;
            break;
        }
      }
      else{
        switch(j){
          case 0:
            p->palette[i][j] = colorData2[i-LFOURFINALSIZE].avg_R2;
            break;
          case 1:
            p->palette[i][j] = colorData2[i-LFOURFINALSIZE].avg_G2;
            break;
          case 2:
            p->palette[i][j] = colorData2[i-LFOURFINALSIZE].avg_B2;
            break;
      }
    }
  }
}
// restart the image pointer at 0
  fseek(in, sizeof(p->hdr), SEEK_SET);
  // iterate through the image again, this time setting pixels to palette indecies
  for (y = p->hdr.height; y-- > 0; ) {
/* Loop over columns from left to right. */
     for (x = 0; p->hdr.width > x; x++) {
    /*
     * Try to read one 16-bit pixel.  On failure, clean up and
     * return NULL.
     */
    if (1 != fread (&pixel, sizeof (pixel), 1, in)) {
      free (p->img);
      free (p);
      (void)fclose (in);
      return NULL;
    }
    // get 2 MSB of each color in each pixel by
    // 1. right shift by 14 to isolate 4 MSB from R
    // 2. right shift by 9 and AND with 11 to isolate 4 MSB from G
    // 3. right shift by 1 and AND with 11 to isolate 4 MSB from B
    // 4. right shift 1. by 4, 2. by 2 and OR them all together to get 4:4:4
    int bitIndex2_ = (((pixel >> 14) << 4) |
            (((pixel >> 9) & 0x3) << 2) |
            ((pixel >> 3) & 0x3));

    // get 4 MSB of each color in each pixel by
    // 1. right shift by 12 to isolate 4 MSB from R
    // 2. right shift by 7 and AND with 1111 to isolate 4 MSB from G
    // 3. right shift by 1 and AND with 1111 to isolate 4 MSB from B
    // 4. right shift 1. by 8, 2. by 4 and OR them all together to get 2:2:@
    int bitIndex4_ = (((pixel >> 12) << 8) |
            (((pixel >> 7) & 0xF) << 4) |
            ((pixel >> 1) & 0xF));
    // remap the indecies to the pixels in each image
    int flag = 0;
    if(palette_idx[bitIndex4_] != -1){
      p->img[p->hdr.width * y + x] = palette_idx[bitIndex4_] + LTWOSIZE;
    }
    else{
      flag = 1;
    }

    if(flag == 1){
      p->img[p->hdr.width * y + x] = bitIndex2_ + PALETTESIZE;
    }
  }
}
    /* All done.  Return success. */
    (void)fclose (in);
    return p;

}


int cmpfunc (const void * a, const void * b) {
  colorDataL4 *colorData1 = (struct colorData_t4 *)a;
  colorDataL4 *colorData2 = (struct colorData_t4 *)b;

    return (colorData2->frequency - colorData1->frequency);
}
