package org.firstinspires.ftc.ftcdevcommon;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.Callable;
import java.util.concurrent.CancellationException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;
import java.util.concurrent.ExecutionException;

public class CommonUtils {

    // Requires Android minSdkVersion 26
    // public static String getLocalDateTimeStamp(LocalDateTime pLocalDateTime) {
    //    return pLocalDateTime.format(DateTimeFormatter.ofPattern("MMddHHmm'_'ssSSS"));
    // }

    public static final SimpleDateFormat dateFormat = new SimpleDateFormat("MMddHHmm'_'ssSSS", Locale.US);
    public static String getDateTimeStamp(Date pDate) {
        return dateFormat.format(pDate);
    }

    // See https://stackoverflow.com/questions/43764036/how-to-convert-the-code-to-use-completablefuture
    // Answer #6 from Holger
    public static <R> CompletableFuture<R> launchAsync(Callable<R> pCallable) {
        CompletableFuture<R> cf = new CompletableFuture<>();
        CompletableFuture.runAsync(() -> {
            try {
                cf.complete(pCallable.call());
            } catch (Throwable ex) {
                cf.completeExceptionally(ex);
            }
        });
        return cf;
    }

    // Tested in the eclipse Testbed 3/16/2020.
    public static <R> R getFutureCompletion(CompletableFuture<R> pCompletable) throws InterruptedException, IOException {
        R retVal = null;
        try {
            retVal = pCompletable.get(); // allows catch clause for CompletionException below (no compiler error)
        }

        // Based on --
        // https://stackoverflow.com/questions/10437890/what-is-the-best-way-to-handle-an-executionexception
        // There are lots of answers but the first seems the most straightforward.
        catch (Throwable t) {
            handleFutureException(t);
        }

        return retVal;
    }

    // If you want to use get with a timeout you must catch the TimeoutException first,
    // deal with it separately, then call here.
    public static void handleFutureException(Throwable t) throws InterruptedException, IOException {

        if ((t instanceof ExecutionException) || (t instanceof CompletionException)) {
            t = t.getCause(); // unwrap original exception
        }

        if (t instanceof CancellationException) {
            // Benign in our environment - part of a normal shutdown
            return;
        }

        if (t instanceof InterruptedException) {
            throw (InterruptedException) t;
        }

        if (t instanceof IOException) {
            throw (IOException) t;
        }

        if (t instanceof Error) {
            throw (Error) t;
        }

        if (t instanceof RuntimeException) {
            throw (RuntimeException) t;
        }

        throw new RuntimeException(t);
    }
}
