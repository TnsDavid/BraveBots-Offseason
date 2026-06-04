package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class MultipleTelemetry implements Telemetry {
    List<Telemetry> telemetries= new LinkedList<>();
    public MultipleTelemetry(Telemetry... telemetries){
        this.telemetries.addAll(Arrays.asList(telemetries));
    }
    @Override
    public Item addData(String caption, String format, Object... args) {
        return null;
    }

    @Override
    public Item addData(String caption, Object value) {
        telemetries.forEach(telemetry -> telemetry.addData(caption, value));
        return null;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return null;
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return null;
    }

    @Override
    public boolean removeItem(Item item) {
        return false;
    }

    @Override
    public void clear() {
        telemetries.forEach(Telemetry::clear);
    }

    @Override
    public void clearAll() {
        telemetries.forEach(Telemetry::clearAll);
    }

    @Override
    public Object addAction(Runnable action) {
        return null;
    }

    @Override
    public boolean removeAction(Object token) {
        return false;
    }

    @Override
    public void speak(String text) {
        telemetries.forEach(telemetry -> telemetry.speak(text));
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        telemetries.forEach(telemetry -> telemetry.speak(text, languageCode, countryCode));
    }

    @Override
    public boolean update() {
        telemetries.forEach(Telemetry::update);
        return false;
    }

    @Override
    public Line addLine() {
        telemetries.forEach(Telemetry::addLine);
        return null;
    }

    @Override
    public Line addLine(String lineCaption) {
        telemetries.forEach(telemetry -> telemetry.addLine(lineCaption));
        return null;
    }

    @Override
    public boolean removeLine(Line line) {
        return false;
    }

    @Override
    public boolean isAutoClear() {
        return false;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        telemetries.forEach(telemetry -> telemetry.setAutoClear(autoClear));
    }

    @Override
    public int getMsTransmissionInterval() {
        return 0;
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        telemetries.forEach(telemetry -> telemetry.setMsTransmissionInterval(msTransmissionInterval));
    }

    @Override
    public String getItemSeparator() {
        return "";
    }

    @Override
    public void setItemSeparator(String itemSeparator) {

    }

    @Override
    public String getCaptionValueSeparator() {
        return "";
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {

    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {

    }

    @Override
    public Log log() {
        return null;
    }
}
