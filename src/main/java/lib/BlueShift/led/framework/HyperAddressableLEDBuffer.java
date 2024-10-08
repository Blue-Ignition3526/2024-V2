package lib.BlueShift.led.framework;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class HyperAddressableLEDBuffer {
  public byte[] buffer;

  /**
   * Constructs a new LED buffer with the specified length.
   *
   * @param length The length of the buffer in pixels
   */
  public HyperAddressableLEDBuffer(int length) {
    buffer = new byte[length * 4];
  }

  /**
   * Sets a specific led in the buffer.
   *
   * @param index the index to write
   * @param r the r value [0-255]
   * @param g the g value [0-255]
   * @param b the b value [0-255]
   */
  public void setRGB(int index, int r, int g, int b) {
    buffer[index * 4] = (byte) b;
    buffer[(index * 4) + 1] = (byte) g;
    buffer[(index * 4) + 2] = (byte) r;
    buffer[(index * 4) + 3] = 0;
  }

  /**
   * Sets a specific led in the buffer.
   *
   * @param index the index to write
   * @param r the r value [0-255]
   * @param g the g value [0-255]
   * @param b the b value [0-255]
   * @param brightness the brightness value [0-255]
   */
  public void setRGB(int index, int r, int g, int b, int brightness) {
    setRGB(
        index,
        (r * brightness) / 255,
        (g * brightness) / 255,
        (b * brightness) / 255
    );
  }

  

  /**
   * Sets a specific led in the buffer.
   *
   * @param index the index to write
   * @param h the h value [0-180)
   * @param s the s value [0-255]
   * @param v the v value [0-255]
   */
  public void setHSV(final int index, final int h, final int s, final int v) {
    if (s == 0) {
      setRGB(index, v, v, v);
      return;
    }

    // The below algorithm is copied from Color.fromHSV and moved here for
    // performance reasons.

    // Loosely based on
    // https://en.wikipedia.org/wiki/HSL_and_HSV#HSV_to_RGB
    // The hue range is split into 60 degree regions where in each region there
    // is one rgb component at a low value (m), one at a high value (v) and one
    // that changes (X) from low to high (X+m) or high to low (v-X)

    // Difference between highest and lowest value of any rgb component
    final int chroma = (s * v) / 255;

    // Because hue is 0-180 rather than 0-360 use 30 not 60
    final int region = (h / 30) % 6;

    // Remainder converted from 0-30 to 0-255
    final int remainder = (int) Math.round((h % 30) * (255 / 30.0));

    // Value of the lowest rgb component
    final int m = v - chroma;

    // Goes from 0 to chroma as hue increases
    final int X = (chroma * remainder) >> 8;

    switch (region) {
      case 0:
        setRGB(index, v, X + m, m);
        break;
      case 1:
        setRGB(index, v - X, v, m);
        break;
      case 2:
        setRGB(index, m, v, X + m);
        break;
      case 3:
        setRGB(index, m, v - X, v);
        break;
      case 4:
        setRGB(index, X + m, m, v);
        break;
      default:
        setRGB(index, v, m, v - X);
        break;
    }
  }

  /**
   * Sets a specific LED in the buffer.
   *
   * @param index The index to write
   * @param color The color of the LED
   */
  public void setLED(int index, Color color) {
    setRGB(index, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  /**
   * Sets a specific LED in the buffer.
   *
   * @param index The index to write
   * @param color The color of the LED
   */
  public void setLED(int index, Color8Bit color) {
    setRGB(index, color.red, color.green, color.blue);
  }

  /**
   * Gets the buffer length.
   *
   * @return the buffer length
   */
  public int getLength() {
    return buffer.length / 4;
  }

  /**
   * Gets the color at the specified index.
   *
   * @param index the index to get
   * @return the LED color at the specified index
   */
  public Color8Bit getLED8Bit(int index) {
    return new Color8Bit(
        buffer[index * 4 + 2] & 0xFF, buffer[index * 4 + 1] & 0xFF, buffer[index * 4] & 0xFF);
  }

  /**
   * Gets the color at the specified index.
   *
   * @param index the index to get
   * @return the LED color at the specified index
   */
  public Color getLED(int index) {
    return new Color(
        (buffer[index * 4 + 2] & 0xFF) / 255.0,
        (buffer[index * 4 + 1] & 0xFF) / 255.0,
        (buffer[index * 4] & 0xFF) / 255.0);
  }

  /**
   * Copies the (this) buffer to the specified buffer.
   * @param dest The buffer to copy to
   * @param offset Starting offset in the destination buffer
   */
  public void copy(HyperAddressableLEDBuffer dest, int offset) {
    System.arraycopy(buffer, 0, dest.buffer, offset * 4, buffer.length);
  }

  /**
   * Copies the (this) buffer to the specified buffer.
   * @param dest The buffer to copy to
   */
  public void copy(HyperAddressableLEDBuffer dest) {
    copy(dest, 0);
  }
}
