/*
 * Developed for Audience as part of the OSP project.
 * 
 * Apache Public Licensed.
 *
 * (C) 2015 HY Research LLC
 *
 *  *************** PREVIEW ******************
 * Design of driver is not finalized.
 */


#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include "iio/events.h"

#define MAX_AXIS 	5
#define MAX_IIO		99
#define MAX_SENSOR	10

#define IIO_DEVICE_DIR	"/sys/bus/iio/devices"

enum {
	axis_x,
	axis_y,
	axis_z,
	axis_r,
	axis_timestamp,
	axis_invalid
};

static const char *axis_name[] = {
	[axis_x] = "X Axis",
	[axis_y] = "Y Axis",
	[axis_z] = "Z Axis",
	[axis_r] = "R Axis",
	[axis_timestamp] = "Time Axis",
	[axis_invalid] = "INVALID",
};

struct DataDesc {
	int sign;
	int size;
	int store;
	int shift;
};

struct IIO_SensorAxis {
	struct DataDesc dd;
	int index;
	int offset;
};

struct IIO_Sensor {
	struct IIO_SensorAxis IIOAxis[MAX_AXIS];
	int index2axis[MAX_AXIS];
	int iionum;
	int rec_sz;
	char *name;
	int fd;
	int evfd;
} IIOSen[MAX_SENSOR];

void usage(void)
{
	fprintf(stderr, "osp-iio [-l| -c COUNT] sensorname ...\n");
}

/* 
 * Find the IIO device number associate with the named sensor
 */
int getiionum(const char *sname)
{
	int i, j;
	FILE *f;
	char fname[PATH_MAX];
	char name[1024];

	for (i = 0; i < MAX_IIO; i++) {
		snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/name", i);
		fname[PATH_MAX-1] = '\0';
		
		f = fopen(fname, "r");
		if (f == NULL) continue;
		if (fgets(name, 1023, f) != NULL) {
			name[1023] = '\0';
			for (j = 0; j < 1024 && name[j] != '\0'; j++) {
				if (name[j] == '\n' ||
					name[j] == '\r') {
					name[j] = '\0';
					break;
				}
			}
		}
		fclose(f);
		if (strcmp(name, sname) == 0)
			return i;
	}
	return -1;
}

/*
 * List all the available IIO devices and the name.
 */
void dumpiio(void)
{
	int i, j;
	FILE *f;
	char fname[PATH_MAX];
	char name[1024];

	for (i = 0; i < MAX_IIO; i++) {
		snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/name", i);
		fname[PATH_MAX-1] = '\0';
		
		f = fopen(fname, "r");
		if (f == NULL) continue;
		if (fgets(name, 1023, f) != NULL) {
			name[1023] = '\0';
			for (j = 0; j < 1024 && name[j] != '\0'; j++) {
				if (name[j] == '\n' ||
					name[j] == '\r') {
					name[j] = '\0';
					break;
				}
			}
		}
		fclose(f);
		fprintf(stderr, "Device: %s\n", name);
	}
}
/* sysfs support */
void sysfs_write_val(const char *path, const int val)
{
	FILE *f;

	f = fopen(path, "w");
	if (f) {
		fprintf(f, "%i", val);
		fclose(f);
	}
}
void sysfs_write_str(const char *path, const char *str)
{
	FILE *f;

	f = fopen(path, "w");
	if (f) {
		fprintf(f, "%s", str);
		fclose(f);
	}
}
/* -------------- */

/* Parse a IIO type file */
void parse_type(const char *path, struct DataDesc *dd)
{
	FILE *f;
	char desc[1024];
	char *ptr = NULL, *ptr2;

	f = fopen(path, "r");
	if (f) {
		ptr = fgets(desc, 1024, f);
		fclose(f);
	}
	if (ptr) {
		/* le:s64/64>>0 */
		/* Ignore endianness for now */
		if (desc[3] == 's') {
			dd->sign = 1;
		} else {
			dd->sign = 0;
		}

		dd->size = strtol(desc+4, &ptr, 0);
		dd->store = strtol(ptr+1, &ptr2, 0);
		if (*ptr2 == '>') {
			dd->shift = strtol(ptr2+2, NULL, 0);
		} else {
			dd->shift = 0;
		}
	}
}

void print_Desc(const struct DataDesc *dd)
{
	printf("Data: sign = %i, shift = %i, store = %i, size = %i\n",
		dd->sign, dd->shift, dd->store, dd->size);
}


/* Expects:
 * in_NAME_AXIS_misc
 */
static const char *tsname = "timestamp";

int parse_axis(const char *name)
{
	int axis = axis_invalid;
	int i, j;

	if (name[0] == 'i' && name[1] == 'n' && name[2] == '_') {
		for (i = 3, j = 0; name[i] != '\0' && i < 1024; i++, j++) {
			if (name[i] == '_') break;
			if (j < 2048) {
				if (name[i] != tsname[j]) 
					j = 2048;
			}
		}
		if (j < 2048) 
			axis = axis_timestamp;
		else if (name[i] == '_') {
			i++;
			switch (name[i]) {
			case 'x':
				axis = axis_x;
				break;
			case 'y':
				axis = axis_y;
				break;
			case 'z':
				axis = axis_z;
				break;
			case 'r':
				axis = axis_r;
				break;
			}
		}
	}

	return axis;
}

/* 
 * Parse the index file.
 */
int parse_index(const char *fname)
{
	FILE *f;
	char desc[1024];
	char *ptr = NULL;
	int idx = -1;

	f = fopen(fname, "r");
	if (f) {
		ptr = fgets(desc, 1024, f);
		desc[1023] = '\0';
		fclose(f);
	}
	if (ptr) {
		idx = atoi(desc);
	}
	return idx;
}
/* 
 * Parse contents of scan elements directory.
 * Enable all elements.
 */
void setupiio(struct IIO_Sensor *is)
{
	char dname[PATH_MAX];
	char fname[PATH_MAX];
	DIR *d;
	struct dirent *dent;
	int len, ax;

	snprintf(dname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/scan_elements", is->iionum);
	
	d = opendir(dname);

	while((dent = readdir(d)) != NULL) {
		if (dent->d_name[0] == '.') continue;
		len = strlen(dent->d_name);
		if (len < 6) continue;

		ax = parse_axis(dent->d_name);
		if (ax < 0) continue;

		if (strcmp(dent->d_name + len - 5, "_type") == 0) {
			snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/scan_elements/%s", is->iionum, dent->d_name);

			parse_type(fname, &is->IIOAxis[ax].dd);
			print_Desc(&is->IIOAxis[ax].dd);
		} else if (strcmp(dent->d_name + len - 3, "_en") == 0) {
			snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/scan_elements/%s", is->iionum, dent->d_name);
			sysfs_write_val(fname, 1);
		} else if (strcmp(dent->d_name + len - 6, "_index") == 0) {
			snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/scan_elements/%s", is->iionum, dent->d_name);
			is->IIOAxis[ax].index = parse_index(fname);
			is->index2axis[is->IIOAxis[ax].index] = ax;
		}
	}

	closedir(d);
	snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/trigger/current_trigger", is->iionum);
	printf("setting trigger name %s to %s\n", is->name, fname);
	sysfs_write_str(fname, is->name);
	snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/buffer/length", is->iionum);
	sysfs_write_val(fname, 100);
	snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/buffer/enable", is->iionum);
	sysfs_write_val(fname, 1);
}

void unsetup_iio(struct IIO_Sensor *is, int sencount)
{
	int i, len;
	char dname[PATH_MAX];
	char fname[PATH_MAX];
	DIR *d;
	struct dirent *dent;

	for (i = 0; i < sencount; i++) {
		snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/buffer/enable", is[i].iionum);
		sysfs_write_val(fname, 0);


		snprintf(dname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/scan_elements", is[i].iionum);
	
		d = opendir(dname);

		while((dent = readdir(d)) != NULL) {
			if (dent->d_name[0] == '.') continue;
			len = strlen(dent->d_name);
			if (len < 6) continue;
			if (strcmp(dent->d_name + len - 3, "_en") == 0) {
				snprintf(fname, PATH_MAX, IIO_DEVICE_DIR"/iio:device%i/scan_elements/%s", is[i].iionum, dent->d_name);
				sysfs_write_val(fname, 0);
			}
		}

		closedir(d);
	}
}

void parse_iiodata(struct IIO_Sensor *is, char *buf, int len)
{
	int i, ax;
	union {
		uint8_t b[8];
		uint16_t s[4];
		uint32_t w[2];
		uint64_t ll[1];
		int8_t sb[8];
		int16_t ss[4];
		int32_t sw[2];
		int64_t sll[1];
	} conf;
	double outval;

	printf("%s:", is->name);
	for (i = 0; i < MAX_AXIS; i++) {
		ax = is->index2axis[i];
		if (ax < 0) continue;
		conf.w[0] = 0; conf.w[1] = 0;

		switch (is->IIOAxis[ax].dd.size) {
		case 8:
			conf.b[0] = buf[is->IIOAxis[ax].offset];
			if (is->IIOAxis[ax].dd.sign)
				outval = conf.sb[0];
			else
				outval = conf.b[0];
			break;
		case 16:
			conf.b[0] = buf[is->IIOAxis[ax].offset];
			conf.b[1] = buf[is->IIOAxis[ax].offset+1];
			if (is->IIOAxis[ax].dd.sign)
				outval = conf.ss[0];
			else
				outval = conf.s[0];
			break;
		case 32:
			conf.b[0] = buf[is->IIOAxis[ax].offset];
			conf.b[1] = buf[is->IIOAxis[ax].offset+1];
			conf.b[2] = buf[is->IIOAxis[ax].offset+2];
			conf.b[3] = buf[is->IIOAxis[ax].offset+3];
			if (is->IIOAxis[ax].dd.sign)
				outval = conf.sw[0];
			else
				outval = conf.w[0];
			break;
		case 64:	
			conf.b[0] = buf[is->IIOAxis[ax].offset];
			conf.b[1] = buf[is->IIOAxis[ax].offset+1];
			conf.b[2] = buf[is->IIOAxis[ax].offset+2];
			conf.b[3] = buf[is->IIOAxis[ax].offset+3];
			conf.b[4] = buf[is->IIOAxis[ax].offset+4];
			conf.b[5] = buf[is->IIOAxis[ax].offset+5];
			conf.b[6] = buf[is->IIOAxis[ax].offset+6];
			conf.b[7] = buf[is->IIOAxis[ax].offset+7];
			if (is->IIOAxis[ax].dd.sign)
				outval = conf.sll[0];
			else
				outval = conf.ll[0];
			break;
		}
		if (is->IIOAxis[ax].dd.shift)
			outval /= (double)(1<<is->IIOAxis[ax].dd.shift);

		if (ax == axis_timestamp)
			printf("%s = 0x%llx | ", axis_name[ax], conf.ll[0]);
		else
			printf("%s = %08f | ", axis_name[ax], outval);
	}
	printf("\n");
}

static void handle_iio_data(struct IIO_Sensor *is, char *buf, int reclen)
{
	int ret;
	int used;

	/* Main sensor data */
	ret = read(is->fd, buf, reclen*5);
	if (ret > 0)
		while (0) {printf("Got data %i\n", ret);}
	else
		return;
	used = 0;
	do {
		parse_iiodata(is, buf+used, ret);
		ret -= is->rec_sz;
		used+= is->rec_sz;
	} while (ret > 0);
}

static void handle_iio_event(struct IIO_Sensor *is)
{
	int ret;
	struct iio_event_data event;

	ret = read(is->fd, &event, sizeof(event));
	if (ret < 0) {
		printf("Error on event channel - %i\n", errno);
	} else {
		printf("Got event for sensor %s\n", is->name);
	}
}

void mainloop(int count, struct IIO_Sensor *is,
		const int sencount, const int reclen)
{
	int fd, ret;
	char dname[PATH_MAX];
	char *buf;
	int i, j;
	struct pollfd pfd[10];
	int nfd = 0;

	buf = malloc(reclen*5);
	if (buf == NULL) return;

	for (i = 0; i < sencount; i++) {
		snprintf(dname, PATH_MAX, "/dev/iio:device%i", is[nfd].iionum);

		fd = open(dname, O_RDONLY);
		if (fd < 0) return;
		is[i].fd = fd;

		ret = ioctl(fd, IIO_GET_EVENT_FD_IOCTL, &is[i].evfd);
		if (ret >= 0) {
			pfd[nfd].fd = is[i].evfd;
			pfd[nfd].events = POLLIN;
			nfd++;
		}

		pfd[nfd].fd = fd;
		pfd[nfd].events = POLLIN;
		nfd++;
	}
	while((count < 0) || count > 0) {
		if (count > 0)
			count--;	

		if (poll(pfd, nfd, -1) <= 0)
			continue;

		for (i = 0; i < nfd; i++) { 
			if (!(pfd[i].revents & POLLIN))
				continue;
			for (j = 0; j < sencount; j++) {	
				if (pfd[i].fd == is[j].fd ) {
					/* Main sensor data */
					handle_iio_data(&is[j], buf, reclen*5);
				} else if (pfd[i].fd == is[j].evfd) {
					/* Events */
					handle_iio_event(&is[j]);
				}
			}
		}
	}
}

int main(int argc, char **argv)
{
	char *sensorname;
	int arg_name = 1;
	int i, j;
	int rec_sz, maxrec = 0;
	int count = -1;
	int sencount = 0;
	int disable = 0;

	if (argc < 2) {
		usage();
		exit(2);
	}
	if (argv[1][0] == '-') {
		arg_name++;
		switch(argv[1][1]) {
		case 'd':
			disable = 1;
			break;
		case 'l':
			dumpiio();
			exit(0);
			break;
		case 'c':
			arg_name++;
			if (argc < arg_name) {
				usage();
				exit(2);
			}
			count = atoi(argv[2]);
			break;	
		}
	}
	do {
		sensorname = argv[arg_name];

		for (i = 0; i < MAX_AXIS; i++) {
			IIOSen[sencount].IIOAxis[i].index = -1;
			IIOSen[sencount].index2axis[i] = -1;
		}
		IIOSen[sencount].fd = -1;
		IIOSen[sencount].evfd = -1;

		IIOSen[sencount].iionum = getiionum(sensorname);
		if (IIOSen[sencount].iionum >= 0) {
			fprintf(stderr, "Dumping %s on %i\n", sensorname, IIOSen[sencount].iionum);
			IIOSen[sencount].name = strdup(sensorname);
			setupiio(&IIOSen[sencount]);
			fflush(stderr);
			sencount++;
		}
		arg_name++;
	} while (arg_name < argc);

	for (j = 0; j < sencount; j++) {
		rec_sz = 0;
		for (i = 0; i < MAX_AXIS; i++) {
			if (IIOSen[j].IIOAxis[i].index >= 0) {
				printf("%s @ %i:\n", axis_name[i], IIOSen[j].IIOAxis[i].index);
				print_Desc(&IIOSen[j].IIOAxis[i].dd);
				/* Make sure each group is aligned */
				printf("1: rec_sz = %i\n", rec_sz);
				if (rec_sz % (IIOSen[j].IIOAxis[i].dd.store/8) != 0) {
					rec_sz += ((IIOSen[j].IIOAxis[i].dd.store/8)-(rec_sz % (IIOSen[j].IIOAxis[i].dd.store/8)));
				}
				IIOSen[j].IIOAxis[i].offset = rec_sz;
				rec_sz += IIOSen[j].IIOAxis[i].dd.store/8;
				printf("2: rec_sz = %i\n", rec_sz);
			}		
		}
		if (rec_sz > maxrec) maxrec = rec_sz;
		IIOSen[j].rec_sz = rec_sz;
		printf("Record size %i\n", rec_sz);
	}
	printf("Total record size: %i\n", maxrec);
	printf("Running count = %i\n", count);

	if (disable) {
		printf("Disabling...\n");
		unsetup_iio(IIOSen, sencount);
	} else {
		mainloop(count, IIOSen, sencount, maxrec);
		unsetup_iio(IIOSen, sencount);
	}

	return 0;	
}

