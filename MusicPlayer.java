/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package Ftc2022FreightFrenzy_3543;

import TrcCommonLib.trclib.TrcSong;
import TrcCommonLib.trclib.TrcSongPlayer;
import TrcFtcLib.ftclib.FtcAndroidTone;

public class MusicPlayer
{
    private static final double ENVELOPE_ATTACK = 0.0;
    private static final double ENVELOPE_DECAY = 0.0;
    private static final double ENVELOPE_SUSTAIN = 1.0;
    private static final double ENVELOPE_RELEASE = 0.02;
    private static final double BAR_DURATION = 1.92;
    //
    // Note string syntax:
    //  <note>[#|b]<octave>.<noteType>[+]{.<noteType>[+]}
    //  where <note>     - 'A' through 'G'
    //        #          - sharp
    //        b          - flat
    //        <octave>   - 1 through 8 (e.g. C4 is the middle C)
    //        <noteType> - note type (1: whole, 2: half, 4: quarter, ...)
    //        +          - add half time
    //
    public static final String starWarsName = "Star Wars";
    private static final String[] starWarsSections =
        {
            // section 1
            "1:G4.12,G4.12,G4.12,"
            + "C5.2,G5.2,"
            + "F5.12,E5.12,D5.12,C6.2,G5.4,"
            + "F5.12,E5.12,D5.12,C6.2,G5.4,"
            + "F5.12,E5.12,F5.12,D5.2,G4.8,G4.8,"
            + "C5.2,G5.2",
            // section 2
            "2:F5.12,E5.12,D5.12,C6.2,G5.4,"
            + "F5.12,E5.12,D5.12,C6.2,G5.4,"
            + "F5.12,E5.12,F5.12,D5.2,G4.8,G4.8,"
            + "A4.4+,A4.8,F5.8,E5.8,D5.8,C5.8,"
            + "C5.12,D5.12,E5.12,D5.4,B4.4,G4.8,G4.8,"
            + "A4.4+,A4.8,F5.8,E5.8,D5.8,C5.8,"
            + "G5.4,D5.2,G4.8,G4.8",
            // section 3
            "3:A4.4+,A4.8,F5.8,E5.8,D5.8,C5.8,"
            + "C5.12,D5.12,E5.12,D5.4,B4.4,G5.8,G5.8,"
            + "C6.8,Bb5.8,Ab5.8,G5.8,F5.8,Eb5.8,D5.8,C5.8,"
            + "G5.2+"
        };
    private final String starWarsSequence = "1,2,3";
    private final TrcSong starWars = new TrcSong("StarWars", starWarsSections, starWarsSequence);
    private boolean starWarsPlaying = false;

    public static final String lesMiserablesName = "Les Miserables";
    private static final String[] lesMiserablesSections =
        {
            // section 1
            "1:A4.8+,G4.16,"
            + "F4.8+,G4.16,A4.8+,Bb4.16,C5.4,A4.12,G4.12,F4.12,"
            + "E4.8+,D4.16,E4.8+,F4.16,C4.4,D4.12,C4.12,Bb3.12,"
            + "A3.8+,C4.16,F4.8+,A4.16,G4.8+,F#4.16,G4.8+,D4.16,"
            + "F4.8+,E4.16,E4.8+,F4.16,G4.4,A4.8+,G4.16",
            // section 2
            "2:F4.8+,G4.16,A4.8+,Bb4.16,C5.4,A4.12,G4.12,F4.12,"
            + "E4.8+,D4.16,E4.8+,F4.16,C4.4,D4.12,C4.12,Bb3.12,"
            + "A3.8+,C4.16,F4.8+,A4.16,G4.12,F#4.12,G4.12,Bb4.8+,E4.16,"
            + "F4.4,R.2,E4.8+,E4.16",
            // section 3
            "3:A4.8+,G#4.16,A4.8+,B4.16,C5.8+,B4.16,A4.8+,C5.16,"
            + "B4.8+,A4.16,G4.8+,A4.16,B4.4,R.12,B4.12,C5.12,"
            + "D5.8+,C5.16,B4.8+,C5.16,D5.8+,C5.16,B4.8+,D5.16,"
            + "C5.8+,B4.16,A4.8+,B4.16,C5.4,R.8+,A4.16",
            // section 4
            "4:C5.12,B4.12,A4.12,C5.12,B4.12,A4.12,C5.12,B4.12,A4.12,C5.12,B4.12,C5.12,"
            + "D5.2,R.4,E5.8+,D5.16,"
            + "C5.8+,D5.16,E5.8+,F5.16,G5.4,E5.12,D5.12,C5.12,"
            + "B4.8+,A4.16,B4.8+,C5.16,G4.4,A4.12,G4.12,F4.12",
            // section 5
            "5:E4.8+,G4.16,C5.8+,E5.16,D5.8+,C#5.16,D5.8+,A4.16,"
            + "C5.8+,B4.16,B4.8+,C5.16,D5.4,E5.8+,D5.16,"
            + "C5.8+,D5.16,E5.8+,F5.16,G5.4,E5.12,D5.12,C5.12,"
            + "B4.8+,A4.16,B4.8+,C5.16,G4.4,A4.12,G4.12,F4.12",
            // section 6
            "6:E4.8+,G4.16,C5.8+,E5.16,D5.12,C#5.12,D5.12,F5.8+,B4.16,"
            + "C5.4,R.2,E4.8+,E4.16",
            // section 7
            "7:E4.8+,G4.16,C5.8+,E5.16,D5.12,C#5.12,D5.12,F5.8+,B4.16,"
            + "C5.4,R.2.4"
        };
    private static final String lesMiserablesSequence = "1,2,3,4,5,6,3,4,5,7";
    private static final TrcSong lesMiserables =
        new TrcSong("LesMiserables", lesMiserablesSections, lesMiserablesSequence);

    private final TrcSongPlayer songPlayer;
    private boolean lesMiserablesPlaying = false;

    public MusicPlayer(FtcAndroidTone androidTone)
    {
        androidTone.setSoundEnvelope(ENVELOPE_ATTACK, ENVELOPE_DECAY, ENVELOPE_SUSTAIN, ENVELOPE_RELEASE);
        androidTone.setSoundEnvelopeEnabled(true);
        songPlayer = new TrcSongPlayer("SongPlayer", androidTone);
    }   //MusicPlayer

    public void toggleSong(String songName)
    {
        if (songName.equals(starWarsName))
        {
            starWarsPlaying = !starWarsPlaying;
            if (starWarsPlaying)
            {
                songPlayer.playSong(starWars, BAR_DURATION, true, false);
                if (lesMiserablesPlaying)
                {
                    lesMiserablesPlaying = false;
                }
            }
            else
            {
                songPlayer.pause();
            }
        }
        else if (songName.equals(lesMiserablesName))
        {
            lesMiserablesPlaying = !lesMiserablesPlaying;
            if (lesMiserablesPlaying)
            {
                songPlayer.playSong(lesMiserables, BAR_DURATION, true, false);
                if (starWarsPlaying)
                {
                    starWarsPlaying = false;
                }
            }
            else
            {
                songPlayer.pause();
            }
        }
    }   //toggleSong

}   //class MusicPlayer
