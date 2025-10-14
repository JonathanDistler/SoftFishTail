from moviepy.editor import VideoFileClip, vfx

# Path to the input video
input_file = r"C:\MuJoCo\Fish_Force_Test_20.mp4"

# Load the video
video = VideoFileClip(input_file)

# Speed up the video (e.g., 88x faster)
sped_up_video = video.fx(vfx.speedx, factor=88)

# Path to save the sped-up video
output_file = r"C:\MuJoCo\mujoco_spedup_video.mp4"

# Write the sped-up video to a new file
sped_up_video.write_videofile(
    output_file,
    codec="libx264",
    audio_codec="aac",
    temp_audiofile="temp-audio.m4a",
    remove_temp=True
)

print(f"Video has been sped up and saved as '{output_file}'")
