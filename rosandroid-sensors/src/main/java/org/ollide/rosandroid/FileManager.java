package org.ollide.rosandroid;

import android.content.res.Resources;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

public class FileManager {
    public static File copyResource(Resources r, int rsrcId, File dstFile) throws IOException {
        InputStream is = r.openRawResource(rsrcId);
        FileOutputStream os = new FileOutputStream(dstFile);
        byte[] buffer = new byte[4096];
        int bytesRead;
        while ((bytesRead = is.read(buffer)) != -1) {
            os.write(buffer, 0, bytesRead);
        }
        os.close();
        is.close();
        return dstFile;
    }
}
