/* (C) Copyright 2013
 * Nikita Kiryanov, Compulab, nikita at compulab.co.il.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <asm/types.h>
#include <bmp_layout.h>
#include <compiler.h>

int bmp_signature_valid(bmp_image_t *bmp)
{
	return bmp->header.signature[0] == 'B' &&
		bmp->header.signature[1] == 'M';
}

__u32 bmp_get_file_size(bmp_image_t *bmp)
{
	return le32_to_cpu(bmp->header.file_size);
}

__u32 bmp_get_data_offset(bmp_image_t *bmp)
{
	return le32_to_cpu(bmp->header.data_offset);
}

__u32 bmp_get_size(bmp_image_t *bmp)
{
	return le32_to_cpu(bmp->header.size);
}

__u32 bmp_get_width(bmp_image_t *bmp)
{
	return le32_to_cpu(bmp->header.width);
}

__u32 bmp_get_height(bmp_image_t *bmp)
{
	return le32_to_cpu(bmp->header.height);
}

__u16 bmp_get_planes(bmp_image_t *bmp)
{
	return le16_to_cpu(bmp->header.planes);
}

__u16 bmp_get_bit_count(bmp_image_t *bmp)
{
	return le16_to_cpu(bmp->header.bit_count);
}

__u32 bmp_get_compression(bmp_image_t *bmp)
{
	return le32_to_cpu(bmp->header.compression);
}

__u32 bmp_get_image_size(bmp_image_t *bmp)
{
	return le32_to_cpu(bmp->header.image_size);
}

__u32 bmp_get_x_pixels_per_m(bmp_image_t *bmp)
{
	return le32_to_cpu(bmp->header.x_pixels_per_m);
}

__u32 bmp_get_y_pixels_per_m(bmp_image_t *bmp)
{
	return le32_to_cpu(bmp->header.y_pixels_per_m);
}

__u32 bmp_get_colors_used(bmp_image_t *bmp)
{
	return le32_to_cpu(bmp->header.colors_used);
}

__u32 bmp_get_colors_important(bmp_image_t *bmp)
{
	return le32_to_cpu(bmp->header.colors_important);
}