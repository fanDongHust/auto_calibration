import os
import cv2

# 要被合成的多张图片所在文件夹
# 路径分隔符最好使用“/”,而不是“\”,“\”本身有转义的意思；或者“\\”也可以。
# 因为是文件夹，所以最后还要有一个“/”
#file_dir = '/home/deliadong/Job/SVS/hank/tesla-move/svs/1-/'
file_dir = '/home/deliadong/Job/SVS/hank/svs4/'
outfile = file_dir+'tesla-move.mp4'

def file_name(path):
    sorted_file_name = []
 
    files = os.listdir(path)  # 采用listdir来读取所有文件
    files.sort(key=lambda x: int(x[:x.find(".")]))  # 按照前面的数字字符排序
    print(files)


# VideoWriter是cv2库提供的视频保存方法，将合成的视频保存到该路径中
# 'MJPG'意思是支持jpg格式图片
# fps = 5代表视频的帧频为5，如果图片不多，帧频最好设置的小一点
# (1280,720)是生成的视频像素1280*720，一般要与所使用的图片像素大小一致，否则生成的视频无法播放
# 定义保存视频目录名称和压缩格式，像素为1280*720
#video = cv2.VideoWriter('/home/deliadong/Job/SVS/hank/tesla-move/svs/1/tesla-move.avi',cv2.VideoWriter_fourcc(*'MJPG'),5,(1280*2,720*2))

#for i in range(1,len(list)):
    #读取图片
#    img = cv2.imread('C:/Users/xxx/Desktop/img/'+list[i-1])     
   	# resize方法是cv2库提供的更改像素大小的方法
    # 将图片转换为1280*720像素大小
#    img = cv2.resize(img,(1280*2,720*2))
    # 写入视频
#    video.write(img)

# 释放资源
#video.release()

fps = 5 # 帧率
first_num = 2474
#34997
num = 2597-2474+1
#278 #文件夹里图片的数量
img_array = []
img_width = 1280*2
img_height = 720*2
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(outfile, fourcc, fps, (img_width, img_height))

for i in range(first_num, first_num+num):
    filename = file_dir+"fourinone_" + str(i) +".jpg"
    img = cv2.imread(filename)
    if img is None:
        print(filename + " is non-existent!")
        continue
    img_array.append(img)
for i in range(len(img_array)):
    out.write(img_array[i])
out.release()
