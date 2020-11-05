#!/usr/bin/python
#coding:utf-8

import os

if __name__ == '__main__':
    file_path = '/home/qk/Desktop/1016/new_colmap/rotation/2020-10-16-15-27-09/mav0/cam0/data'
    csv_path = file_path + '.csv'

    f = open(csv_path)
    lines = f.readlines()
    num = 0
    for line in lines:
        num = num + 1
        image_name = line.split(',')[1]
        image_name = image_name.split('\n')[0]
        os.rename(file_path + '/' + image_name, file_path + '/' + str(num) + '.png')