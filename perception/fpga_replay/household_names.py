"""Class order of the versioned household9 detector; independent of COCO indices."""
NAMES = ('remote', 'medicine bottle', 'medicine box', 'key', 'glasses',
         'waste paper', 'drink can', 'plastic packaging', 'cell phone')
CLASS_ZH = {'remote': '遥控器', 'medicine bottle': '药瓶', 'medicine box': '药盒',
            'key': '钥匙', 'glasses': '眼镜', 'waste paper': '废纸',
            'drink can': '饮料罐', 'plastic packaging': '塑料包装', 'cell phone': '手机'}
ALIASES_ZH = {'medicine bottle': ('药品瓶',), 'medicine box': ('药品盒',),
              'waste paper': ('废纸团', '纸团'), 'drink can': ('易拉罐',),
              'plastic packaging': ('塑料包装袋',)}

NAMES33 = NAMES + (
    'backpack', 'umbrella', 'handbag', 'sports ball', 'bottle', 'wine glass', 'cup', 'spoon', 'bowl',
    'banana', 'apple', 'orange', 'chair', 'potted plant', 'dining table', 'laptop', 'mouse', 'keyboard',
    'book', 'clock', 'vase', 'scissors', 'teddy bear', 'earbud case')
# classes this model serves in `combined33`; the other COCO classes stay with the COCO engine
PRIMARY33 = NAMES + ('mouse', 'earbud case')
EXTRA_ZH33 = {'earbud case': '耳机充电盒'}
