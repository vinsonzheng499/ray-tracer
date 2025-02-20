from PIL import Image

def convert_rgba_to_rgb(input_png_path, output_png_path):
    with Image.open(input_png_path) as img:
        rgb_img = img.convert("RGB")
        rgb_img.save(output_png_path, format="PNG")

if __name__ == "__main__":
    input_png = ""
    output_png = ""

    convert_rgba_to_rgb(input_png, output_png)
    print(f"Converted {input_png} to RGB and saved as {output_png}")
