bbox1 = ((1, 2), (3, 4)), ((5, 6), (7, 8))
bbox2 = ((9, 12), (13, 14)), ((15, 16), (17, 18))
bbox3 = ((19, 212), (213, 214)), ((215, 216), (217, 218))

class BoxKeeper:

	def __init__(self):
		self._list = []
		self._maxLen = 1

	def set_max_history(self,l):
		self._maxLen = l

	def add_boxes(self,bboxes):
		self._list.append(bboxes)
		if len(self._list) > self._maxLen:
			self._list.pop(0)

	def get_boxes(self):
		bb = []
		for item in self._list:
			bb += item
		return bb

keeper = BoxKeeper()
keeper.set_max_history(2)
keeper.add_boxes(bbox1)
print(keeper.get_boxes())
keeper.add_boxes(bbox2)
print(keeper.get_boxes())
keeper.add_boxes(bbox3)
print(keeper.get_boxes())
bb = keeper.get_boxes()
print(bb[0])
