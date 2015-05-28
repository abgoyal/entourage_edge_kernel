#ifndef KVPAIRS_H
#define KVPAIRS_H

/*
 * This structure hold the units mutable configuration as a collection
 * of key-value pairs.
*/

	struct keyValuePair
		{
		char key[32];

		enum
				{
				end_of_list = 0,
				dec_integer,
				hex_integer,
				hex_short,
				string,
				byte_string,
				hidden,		// types beyond this cannot be displayed using show_all
				show_all,
				save_all,
				none,
				} type;

		union
				{
				int asInteger[8];
				int asShort[16];
				char asString[32];
				} value;
		};

#define KV_PAIR_ORIGIN 0xbaadf00d
#define KV_PAIR_MAGIC 0xbaadf010

	struct nvramStruct
		{
		unsigned magic;
	 	char secret_buffer[1024];
	    struct keyValuePair kvPairs[32];
		} nvramData	= {
			.magic = KV_PAIR_MAGIC,
			.secret_buffer = 
				{ "42\n" },

			.kvPairs = {
				{
				.key = "KNOWN_QUANTITY",
				.type = dec_integer,
				.value.asInteger = { 42, }
				},

				{
				.key = "HEX_QUANTITY",
				.type = hex_integer,
				.value.asInteger = { 0x42, }
				},

				{
				.key = "SERIAL_NUMBER",
				.type = string,
				.value.asString = "none",
				},

				{
				.key = "SKU",
				.type = string,
				.value.asString = "100-95-00002-01",
				},

				{
				.key = "SHOW_ALL",
				.type = show_all,
				},
			
				{
				.key = "SAVE_ALL",
				.type = save_all,
				},
			
				{
	   			.type = end_of_list,
				},

			},
		};
#endif
