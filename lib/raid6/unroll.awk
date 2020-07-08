
# This filter requires one command line option of form -vN=n
# where n must be a decimal number.
#
# Repeat each input line containing $$ n times, replacing $$ with 0...n-1.
# Replace each $# with n, and each $* with a single $.

BEGIN {
	n = N + 0
}
{
	if (/\$\$/) { rep = n } else { rep = 1 }
	for (i = 0; i < rep; ++i) {
		tmp = $0
		gsub(/\$\$/, i, tmp)
<<<<<<< HEAD
		gsub(/\$\#/, n, tmp)
=======
		gsub(/\$#/, n, tmp)
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
		gsub(/\$\*/, "$", tmp)
		print tmp
	}
}
