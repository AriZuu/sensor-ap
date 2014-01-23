#
# Copyright (c) 2012-2013, Ari Suutari <ari@stonepile.fi>.
# All rights reserved. 
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. The name of the author may not be used to endorse or promote
#     products derived from this software without specific prior written
#     permission. 
# 
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
# INDIRECT,  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.

#
# Create patch for bsp_external files.
#
ORIG_BSP =	../../../simpliciti/bsp/boards
bsp-diff:
	diff -u $(ORIG_BSP)/EZ430RF/bsp_external/mrfi_board.c \
		mrfi_board.c > mrfi_board.patch || true
	diff -u $(ORIG_BSP)/EZ430RF/bsp_external/mrfi_board_defs.h \
		mrfi_board_defs.h > mrfi_board_defs.patch || true

all:	mrfi_board_defs.h mrfi_board.c

clean:

mrfi_board_defs.h:
	patch -o mrfi_board_defs.h $(ORIG_BSP)/EZ430RF/bsp_external/mrfi_board_defs.h < mrfi_board_defs.patch

mrfi_board.c:
	patch -o mrfi_board.c $(ORIG_BSP)/EZ430RF/bsp_external/mrfi_board.c < mrfi_board.patch
