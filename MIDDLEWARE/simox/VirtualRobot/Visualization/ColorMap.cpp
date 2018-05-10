/**
* @package    VirtualRobot
* @author     David Gonzales, Nikolaus Vahrenkamp
* @copyright  2011 David Gonzales, Nikolaus Vahrenkamp
*/


#include "ColorMap.h"


namespace VirtualRobot
{



    ColorMap::ColorMap(type t)
    {
        create(t);
    }

    ColorMap::ColorMap()
    {
    }

    ColorMap::~ColorMap()
    {

    }
    void ColorMap::create(type t)
    {
        colorKeys.clear();
        float DeltaPosition;
        float Position;
        colorMapType = t;

        switch (t)
        {
            case eIntensity:
                addColorKey(0, 0, 0, 255, 0.0f);
                addColorKey(255, 255, 255, 255, 1.0f);
                break;

            case eHotAlpha:
                DeltaPosition = 1.0 / 16.0;
                Position = 0.0;
                addColorKey(0, 0, 189, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(0, 0, 255, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(0, 66, 255, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(0, 132, 255, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(0, 189, 255, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(0, 255, 255, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(66, 255, 189, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(132, 255, 132, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(189, 255, 66, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(255, 255, 0, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(255, 189, 0, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(255, 132, 0, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(255, 66, 0, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(255, 0, 0, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(189, 0, 0, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(132, 0, 0, (unsigned char)(255.0 * Position), Position);
                Position += DeltaPosition;
                addColorKey(128, 0, 0, (unsigned char)(255.0 * Position), Position);
                break;

            case eHot:
                DeltaPosition = 1.0 / 16.0;
                Position = -DeltaPosition;
                addColorKey(0, 0, 189, 255, Position += DeltaPosition);
                addColorKey(0, 0, 255, 255, Position += DeltaPosition);
                addColorKey(0, 66, 255, 255, Position += DeltaPosition);
                addColorKey(0, 132, 255, 255, Position += DeltaPosition);
                addColorKey(0, 189, 255, 255, Position += DeltaPosition);
                addColorKey(0, 255, 255, 255, Position += DeltaPosition);
                addColorKey(66, 255, 189, 255, Position += DeltaPosition);
                addColorKey(132, 255, 132, 255, Position += DeltaPosition);
                addColorKey(189, 255, 66, 255, Position += DeltaPosition);
                addColorKey(255, 255, 0, 255, Position += DeltaPosition);
                addColorKey(255, 189, 0, 255, Position += DeltaPosition);
                addColorKey(255, 132, 0, 255, Position += DeltaPosition);
                addColorKey(255, 66, 0, 255, Position += DeltaPosition);
                addColorKey(255, 0, 0, 255, Position += DeltaPosition);
                addColorKey(189, 0, 0, 255, Position += DeltaPosition);
                addColorKey(132, 0, 0, 255, Position += DeltaPosition);
                addColorKey(128, 0, 0, 255, Position += DeltaPosition);
                break;

            case eRed:
                addColorKey(0, 0, 0, 255, 0.0f);
                addColorKey(255, 0, 0, 255, 1.0f);
                break;

            case eRedAlpha:
                addColorKey(0, 0, 0, 0, 0.0f);
                addColorKey(255, 0, 0, 255, 1.0f);
                break;

            /*case eGrayToRed:
                addColorKey(64, 64, 64, 255, 0.0f);
                addColorKey(255, 0, 0, 255, 1.0f);
                break;*/
            case eGreen:
                addColorKey(0, 0, 0, 255, 0.0f);
                addColorKey(0, 255, 0, 255, 1.0f);
                break;

            case eGreenAlpha:
                addColorKey(0, 0, 0, 0, 0.0f);
                addColorKey(0, 255, 0, 255, 1.0f);
                break;

            case eBlue:
                addColorKey(0, 0, 0, 255, 0.0f);
                addColorKey(0, 0, 255, 255, 1.0f);
                break;

            case eBlueAlpha:
                addColorKey(0, 0, 0, 0, 0.0f);
                addColorKey(0, 0, 255, 255, 1.0f);
                break;
        }

        sort();
    }



    bool ColorMap::addColorKey(const unsigned char R, const unsigned char G, const unsigned char B, const unsigned char A, const float Position)
    {
        if ((Position < 0.0) || (Position > 1.0))
        {
            return false;
        }

        ColorKey Key;
        Key.position = Position;
        Key.color.r = R;
        Key.color.g = G;
        Key.color.b = B;
        Key.color.transparency = A;
        colorKeys.push_back(Key);
        sort();
        return true;
    }

    void ColorMap::sort()
    {
        size_t size = colorKeys.size();

        if (size > 1)
        {
            std::sort(colorKeys.begin(), colorKeys.end(), ColorMap::CompareColorKey);
            intervals.clear();

            for (size_t i = 0; i < size; i++)
            {
                colorKeys[i].index = i;
                intervals.push_back(colorKeys[i].position);
            }
        }
    }

    bool ColorMap::CompareColorKey(const ColorMap::ColorKey& lhs, const ColorMap::ColorKey& rhs)
    {
        return lhs.position < rhs.position;
    }

    VirtualRobot::VisualizationFactory::Color ColorMap::getColor(float position) const
    {
        VirtualRobot::VisualizationFactory::Color c;
        getColor(position, c);
        return c;
    }

    bool ColorMap::getColor(float position, VirtualRobot::VisualizationFactory::Color& storeColor) const
    {
        if ((position >= 0.0) && (position <= 1.0))
        {
            VirtualRobot::VisualizationFactory::Color colorA, colorB;
            float FA, FB, FT;

            for (unsigned int i = 0, j = 1; j < intervals.size(); i++, j++)
                if ((position >= intervals[i]) && (position <= intervals[j]))
                {
                    colorA = colorKeys[i].color;
                    colorB = colorKeys[j].color;
                    FA = fabs(intervals[i] - position);
                    FB = fabs(intervals[j] - position);
                    FT = FA + FB;

                    if (FT != 0)
                    {
                        FA /= FT;
                        FB /= FT;
                    }

                    storeColor.r = ((float(colorA.r) * FB) + (float(colorB.r) * FA)) / 255.0f;  // from [0...255] -> [0..1]
                    storeColor.g = ((float(colorA.g) * FB) + (float(colorB.g) * FA)) / 255.0f;
                    storeColor.b = ((float(colorA.b) * FB) + (float(colorB.b) * FA)) / 255.0f;
                    // convert form alpha to transparency!
                    storeColor.transparency = 1.0f - (((float(colorA.transparency) * FB) + (float(colorB.transparency) * FA)) / 255.0f);
                    return true;
                }
        }

        storeColor = VirtualRobot::VisualizationFactory::Color::None();
        return false;
    }

    ColorMap ColorMap::customColorMap(std::vector< VirtualRobot::VisualizationFactory::Color > colors)
    {
        VR_ASSERT(colors.size() > 0);
        ColorMap c;

        c.addColorKey((unsigned char)(colors[0].r * 255.0f + 0.5f), (unsigned char)(colors[0].g * 255.0f + 0.5f), (unsigned char)(colors[0].b * 255.0f + 0.5f), 255 - (unsigned char)(colors[0].transparency * 255.0f + 0.5f), 0.0f);

        if (colors.size() == 1)
        {
            c.addColorKey((unsigned char)(colors[0].r * 255.0f + 0.5f), (unsigned char)(colors[0].g * 255.0f + 0.5f), (unsigned char)(colors[0].b * 255.0f + 0.5f), 255 - (unsigned char)(colors[0].transparency * 255.0f + 0.5f), 1.0f);
            return c;
        }

        float step = 1.0f / (float)(colors.size() - 1);
        float act = 0.0f;

        for (size_t i = 1; i < colors.size(); i++)
        {
            act += step;
            c.addColorKey((unsigned char)(colors[i].r * 255.0f + 0.5f), (unsigned char)(colors[i].g * 255.0f + 0.5f), (unsigned char)(colors[i].b * 255.0f + 0.5f), 255 - (unsigned char)(colors[i].transparency * 255.0f + 0.5f), act);
        }

        return c;
    }




} // namespace VirtualRobot
