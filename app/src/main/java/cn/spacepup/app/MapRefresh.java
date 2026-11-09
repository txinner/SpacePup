package cn.spacepup.app;

/** Schedules work on new camera frames. Times are in camera timestamp nanoseconds. */
final class MapRefresh {
    enum Work { NONE, LIGHT, FULL }

    private static final long CHECK_INTERVAL = 5_000_000_000L;
    private static final long FULL_WINDOW = 2_000_000_000L;
    private static final long FULL_INTERVAL = 220_000_000L;
    boolean inside;
    private boolean hasLocation;
    private long nextCheck, nextFrame, fullUntil;

    void reset(long now, boolean inside) {
        this.inside=inside;
        hasLocation=true;
        finish(now);
    }

    void startFull(long now) {
        fullUntil=now+FULL_WINDOW;
        nextFrame=now;
        nextCheck=now+CHECK_INTERVAL;
    }

    void finish(long now) {
        fullUntil=0;
        nextCheck=now+CHECK_INTERVAL;
    }

    Work next(long now, boolean hasPose, boolean inCoverage) {
        if(hasPose) {
            boolean entered=!inside && inCoverage;
            hasLocation=true;
            inside=inCoverage;
            if(!inside) {
                fullUntil=0;
                return Work.NONE;
            }
            if(entered)startFull(now);
        } else if(hasLocation) {
            hasLocation=false;
            inside=false;
            startFull(now);
        }
        if(now<fullUntil) {
            if(now<nextFrame)return Work.NONE;
            nextFrame=now+FULL_INTERVAL;
            return Work.FULL;
        }
        if(now<nextCheck)return Work.NONE;
        nextCheck=now+CHECK_INTERVAL;
        return Work.LIGHT;
    }
}
