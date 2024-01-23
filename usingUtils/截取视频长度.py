from moviepy.video.io.VideoFileClip import VideoFileClip

# 视频文件路径
video_file_path = './videoBase/videoAll/3.mp4'

# 截取时间段：开始时间和结束时间（以秒为单位）
start_time = 4  # 开始时间，例如 10 秒
end_time = 35    # 结束时间，例如 15 秒

# 加载视频
video = VideoFileClip(video_file_path).subclip(start_time, end_time)

# 输出截取的视频
output_file_path = './videoBase/changeVideo/output3.mp4'
video.write_videofile(output_file_path, codec='mpeg4', audio_codec='aac')

if __name__ == "__main__":
    print("执行视频截取")