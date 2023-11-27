from PIL import Image, ImageDraw, ImageFont

def generate_image(n, text, text_color, background_color, font_size, image_size=(100, 100)):
    # 画像の生成
    image = Image.new('RGB', image_size, color=background_color)
    draw = ImageDraw.Draw(image)

    # フォントの指定
    font_path = 'ヒラギノ丸ゴ ProN W4.ttc'  # デフォルトのフォントのパスを取得
    font = ImageFont.truetype(font_path, size=font_size)  # フォントを新しいサイズで再作成

    # テキストを描画
    text_width, text_height = draw.textsize(text, font)
    text_position = ((image_size[0] - text_width) // 2, (image_size[1] - text_height) // 2)
    draw.text(text_position, text, font=font, fill=text_color)

    # 画像を保存または表示
    image.save('test/img/' +  str(n) + '.png')
    image.show()

if __name__ == "__main__":
    # 生成する画像の設定
    text_to_display = "万"
    text_color = (255, 255, 255)  # 白色
    background_color = (0, 0, 0)  # 黒色
    font_size = 100  # フォントサイズを指定

    # 画像の生成
    generate_image(4, text_to_display, text_color, background_color, font_size)
