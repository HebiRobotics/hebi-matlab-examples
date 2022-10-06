function setupControllerLayout(phoneGroupIO)
% IO Command Setup - Controller Layout
    phoneGroupIO.initializeUI();
    unicode = @(input) sprintf(strrep(input, '\u', '\x'));
    
    phoneGroupIO.setAxisSnap([3 5], [0 0]);
    phoneGroupIO.setAxisValue(6, -1);
    phoneGroupIO.setAxisLabel([3], {unicode('\uD83D\uDD26')});
    phoneGroupIO.setButtonIndicator([1 2 3 4 5 6 7 8], [false false true false false false false true]);
    phoneGroupIO.setButtonLabel([3 7 8], {'Fld', 'Home', unicode('\u23F9')});
    phoneGroupIO.clearText();
end

